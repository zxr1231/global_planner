#pragma once
#include <global_planner/routeCandidates.h>
#include <global_planner/pathGainEvaluator.h>
#include <algorithm>
#include <functional>
#include <tuple>
#include <iterator>

namespace globalPlanner { namespace opportunityRoutes {
using namespace routeCandidates;
enum class Mode { MOTION, DIVERSE, OBSERVATION };
using Observe=std::function<std::vector<uint32_t>(const Sequence&)>;
struct Cancelled:std::runtime_error {Cancelled():std::runtime_error("generation_cancelled") {}};
struct Config {size_t actions=12000, layerQueries=32, beamWidth=8, output=5;};
struct Record {
 Sequence path;size_t segment=0,layer=1;Motion motion;
 std::vector<uint32_t> observed;uint64_t newVoxels=0,lostVoxels=0;double proxyUtility=0;
};
struct Result {std::vector<Record> routes;size_t actions=0,firstFeasible=0,secondFeasible=0,coarseQueries=0;bool operationCutoff=false;std::vector<Sequence> beam;};
inline auto key(const Record& r){return std::make_tuple(r.motion.time,r.motion.length,r.path);}
inline uint64_t difference(const std::vector<uint32_t>& a,const std::vector<uint32_t>& b){
 size_t i=0,j=0;uint64_t n=0;while(i<a.size()){while(j<b.size()&&b[j]<a[i])++j;if(j==b.size()||a[i]!=b[j])++n;++i;}return n;
}
inline void checkSet(const std::vector<uint32_t>& v){if(!std::is_sorted(v.begin(),v.end())||std::adjacent_find(v.begin(),v.end())!=v.end())throw std::invalid_argument("noncanonical observation set");}
inline void annotate(Record& r,const std::vector<uint32_t>& reference,const std::vector<uint32_t>& observed){
 checkSet(reference);checkSet(observed);r.observed=observed;r.newVoxels=difference(observed,reference);r.lostVoxels=difference(reference,observed);
 if(r.motion.time<=1e-6||!std::isfinite(r.motion.time))throw std::invalid_argument("invalid estimated time");
 r.proxyUtility=double(observed.size())/r.motion.time;
}
// Full-path union is independent for each mutually exclusive candidate.
inline std::vector<PathGainWaypoint> samplePath(const Graph& graph,const Sequence& path,double initial,double terminal,double spacing=.25,double yawSpacing=.1){
 if(path.empty()||!std::isfinite(initial)||!std::isfinite(terminal)||!std::isfinite(spacing)||spacing<=0||!std::isfinite(yawSpacing)||yawSpacing<=0)throw std::invalid_argument("invalid sampler");
 std::vector<PathGainWaypoint> out;double yaw=initial;
 auto pos=[&](Id id){const auto& p=graph.positions().at(id);return Eigen::Vector3d(p[0],p[1],p[2]);};
 auto append=[&](const Eigen::Vector3d& p,double a){if(!out.empty()&&(out.back().position-p).norm()<=1e-12&&angleDistance(out.back().yaw,a)<=1e-12)return;out.push_back({p,a});};
 auto rotate=[&](const Eigen::Vector3d& p,double target){double d=std::remainder(target-yaw,2*std::acos(-1.));int n=std::ceil(std::abs(d)/yawSpacing);double begin=yaw;for(int i=1;i<=n;++i)append(p,begin+d*i/n);yaw=target;append(p,yaw);};
 append(pos(path.front()),yaw);
 for(size_t k=1;k<path.size();++k){auto a=pos(path[k-1]),b=pos(path[k]);double length=(b-a).norm();if(length<=1e-12)continue;double heading=std::atan2(b.y()-a.y(),b.x()-a.x());rotate(a,heading);int n=std::ceil(length/spacing);for(int i=1;i<=n;++i){Eigen::Vector3d p;for(int j=0;j<3;++j)p[j]=a[j]+(b[j]-a[j])*i/n;append(p,heading);}}
 rotate(pos(path.back()),terminal);return out;
}
inline double distance(const Graph& graph,Id p,Id q){const auto& a=graph.positions().at(p);const auto& b=graph.positions().at(q);return std::hypot(std::hypot(a[0]-b[0],a[1]-b[1]),a[2]-b[2]);}
inline double separation(const Graph& graph,const Sequence& a,const Sequence& b){
 // Symmetric discrete Hausdorff separation of waypoint positions; geometric control only.
 auto directed=[&](const Sequence& x,const Sequence& y){double result=0;for(auto p:x){double nearest=INFINITY;for(auto q:y)nearest=std::min(nearest,distance(graph,p,q));result=std::max(result,nearest);}return result;};
 return std::max(directed(a,b),directed(b,a));
}
inline std::vector<Record> retain(const Graph& graph,const Sequence& reference,std::vector<Record> rows,Mode mode,size_t count){
 std::sort(rows.begin(),rows.end(),[](const Record& a,const Record& b){return key(a)<key(b);});std::vector<Record> out;
 while(!rows.empty()&&out.size()<count){size_t best=0;double bestScore=-INFINITY;
  for(size_t i=0;i<rows.size();++i){double score=mode==Mode::OBSERVATION?rows[i].proxyUtility:0;
   if(mode==Mode::DIVERSE){score=separation(graph,rows[i].path,reference);for(auto& r:out)score=std::min(score,separation(graph,rows[i].path,r.path));}
   if(score>bestScore){best=i;bestScore=score;}
  }out.push_back(std::move(rows[best]));rows.erase(rows.begin()+best);
 }return out;
}
inline std::vector<Record> spread(std::vector<Record> rows,size_t limit){
 std::sort(rows.begin(),rows.end(),[](const Record& a,const Record& b){return key(a)<key(b);});
 if(rows.size()<=limit)return rows;
 std::vector<Record> out;for(size_t j=0;j<limit;++j)out.push_back(std::move(rows[j*(rows.size()-1)/(limit-1)]));return out;
}
inline Result generate(const Graph& graph,const Sequence& reference,double initial,double terminal,const MotionConfig& motion,
                       Mode mode,const LineCheck& freeLine,const Observe& observe,const Continue& proceed,Config config={}){
 if(!config.actions||config.layerQueries<2||!config.beamWidth||!config.output||reference.size()<2)throw std::invalid_argument("invalid generation configuration");
 validate(motion,Limits{});Result out;auto baseline=measure(graph,reference,initial,terminal,motion);
 if(baseline.time<=1e-6||baseline.length<=1e-6)throw std::invalid_argument("degenerate reference");
 auto check=[&]{if(!proceed())throw Cancelled{};};
 auto isNew=[&](Id node,const Sequence& path){for(auto id:path)if(distance(graph,node,id)<=1e-6)return false;return true;};
 auto attempt=[&](Sequence path,size_t e,size_t layer,std::vector<Record>& target){
  check();if(out.actions==config.actions){out.operationCutoff=true;return false;}++out.actions;
  Record r;r.path=std::move(path);r.segment=e;r.layer=layer;r.motion=measure(graph,r.path,initial,terminal,motion);std::vector<std::string> errors;motionRejections(r.motion,baseline,Limits{},"final",errors);
  if(errors.empty())for(size_t k=1;k<r.path.size();++k){check();if(!freeLine(r.path[k-1],r.path[k])){errors.push_back("unsafe");break;}}
  if(errors.empty())target.push_back(std::move(r));
  return true;
 };
 std::vector<Record> first;
 for(size_t e=0;e+1<reference.size();++e){for(Id u=0;u<graph.size();++u){if(!isNew(u,reference))continue;auto path=reference;path.insert(path.begin()+e+1,u);if(!attempt(std::move(path),e,1,first))break;}if(out.operationCutoff)break;}
 out.firstFeasible=first.size();first=spread(std::move(first),config.layerQueries);
 std::vector<uint32_t> baseSet;
 if(mode==Mode::OBSERVATION&&!first.empty()){check();baseSet=observe(reference);++out.coarseQueries;for(auto& r:first){check();annotate(r,baseSet,observe(r.path));++out.coarseQueries;}}
 auto beam=retain(graph,reference,first,mode,config.beamWidth);for(auto& b:beam)out.beam.push_back(b.path);
 std::vector<Record> second;
 for(const auto& parent:beam){size_t insert=parent.segment+2;for(Id v=0;v<graph.size();++v){if(!isNew(v,parent.path))continue;auto path=parent.path;path.insert(path.begin()+insert,v);if(!attempt(std::move(path),parent.segment,2,second))break;}if(out.operationCutoff)break;}
 out.secondFeasible=second.size();second=spread(std::move(second),config.layerQueries);
 if(mode==Mode::OBSERVATION)for(auto& r:second){check();annotate(r,baseSet,observe(r.path));++out.coarseQueries;}
 first.insert(first.end(),std::make_move_iterator(second.begin()),std::make_move_iterator(second.end()));
 // Geometry aliases are excluded before the expensive full score cap.
 std::vector<Record> unique;for(auto& r:first){bool dup=false;for(const auto& old:unique)if(sameGeometry(graph,r.path,old.path))dup=true;if(!dup)unique.push_back(std::move(r));}
 out.routes=retain(graph,reference,std::move(unique),mode,config.output);check();return out;
}
}}
