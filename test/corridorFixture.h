#pragma once
#include "twoStageFixture.h"
#include "fixtureAllocation.h"
#include <map>
#include <tuple>
namespace corridorFixture {
using namespace twoStageFixture;
constexpr double eps=1e-6;
struct Record {size_t e=0;Id u=0,v=0;Sequence path;size_t exit=0;Motion motion;bool measured=false;std::vector<std::string> reasons;};
struct Pool {Motion reference;std::vector<Record> records;std::vector<size_t> ordered,picked;size_t possible=0;bool truncated=false,skipped=false;};
inline auto key(const Record& r){return std::make_tuple(r.motion.time,r.motion.length,r.u,r.v,r.e);}
inline double segmentDistance(const Eigen::Vector3d& x,const Eigen::Vector3d& a,const Eigen::Vector3d& b){auto d=(b-a).eval();double t=d.squaredNorm()?std::max(0.,std::min(1.,(x-a).dot(d)/d.squaredNorm())):0;return (x-a-t*d).norm();}
inline Pool generate(const Input& in,size_t cap=12000){
 Pool out;if(!in.accepted){out.skipped=true;return out;}
 validate(in.motion,Limits{});auto p=prepareContext(in);out.reference=p->reference.simplifiedMotion;
 for(size_t i=1;i<in.reference.size();++i)if((position(in,in.reference[i])-position(in,in.reference[i-1])).norm()<=eps)throw ContextError("DEGENERATE_REFERENCE_SEGMENT");
 std::set<Edge> arcs;for(auto a:in.edges)if(a.first && a.second && a.first!=a.second)arcs.insert(a);
 out.possible=(in.reference.size()-1)*arcs.size();std::map<Edge,size_t> best;
 for(size_t e=0;e+1<in.reference.size();++e)for(auto arc:arcs){
  if(out.records.size()>=cap){out.truncated=out.possible>cap;goto finish;}
  Record r;r.e=e;r.u=arc.first;r.v=arc.second;
  auto a=position(in,in.reference[e]),b=position(in,in.reference[e+1]),u=position(in,r.u),v=position(in,r.v);
  bool mergeU=(u==a),mergeV=(v==b);
  if(((u-a).norm()<=eps && !mergeU)||((v-b).norm()<=eps && !mergeV))r.reasons.push_back("NEAR_BOUNDARY_AMBIGUITY");
  if((u-v).norm()<=eps)r.reasons.push_back("ZERO_EDGE");
  for(size_t j=0;j<in.reference.size();++j){
   auto q=position(in,in.reference[j]);
   if((u-q).norm()<=eps && !(j==e && mergeU))r.reasons.push_back("NONADJACENT_U_DUPLICATE");
   if((v-q).norm()<=eps && !(j==e+1 && mergeV))r.reasons.push_back("NONADJACENT_V_DUPLICATE");
  }
  r.path.assign(in.reference.begin(),in.reference.begin()+e+1);
  if(!mergeU)r.path.push_back(r.u);
  if(!mergeV)r.path.push_back(r.v);
  r.exit=r.path.size();r.path.insert(r.path.end(),in.reference.begin()+e+1,in.reference.end());
  double chain=(u-a).norm()+(v-u).norm()+(b-v).norm();
  if(segmentDistance(u,a,b)<=eps && segmentDistance(v,a,b)<=eps && chain-(b-a).norm()<=eps)r.reasons.push_back("NO_GEOMETRIC_CHANGE");
  if(r.reasons.empty()){
   r.motion=measure(*p->graph,r.path,in.initialYaw,in.terminalYaw,in.motion);r.measured=true;
   if(r.motion.time<=eps||r.motion.length<=eps)r.reasons.push_back("DEGENERATE_CANDIDATE");
   motionRejections(r.motion,out.reference,Limits{},"final",r.reasons);
   if(!p->view->freeLine(a,u))r.reasons.push_back("incoming_unsafe");
   if(!p->view->freeLine(u,v))r.reasons.push_back("edge_unsafe");
   if(!p->view->freeLine(v,b))r.reasons.push_back("outgoing_unsafe");
  }
  size_t idx=out.records.size();out.records.push_back(r);
  if(r.reasons.empty()){
   auto old=best.find(arc);
   if(old==best.end())best[arc]=idx;
   else if(std::tie(r.motion.time,r.e)<std::tie(out.records[old->second].motion.time,out.records[old->second].e)){
    out.records[old->second].reasons.push_back("edge_insertion_not_best");best[arc]=idx;
   }else out.records[idx].reasons.push_back("edge_insertion_not_best");
  }
 }
 finish:
 std::vector<size_t> sorted;for(auto b:best)sorted.push_back(b.second);
 std::sort(sorted.begin(),sorted.end(),[&](size_t a,size_t b){return key(out.records[a])<key(out.records[b]);});
 for(auto idx:sorted){bool dup=false;for(auto old:out.ordered)if(sameGeometry(*p->graph,out.records[idx].path,out.records[old].path))dup=true;
  if(dup)out.records[idx].reasons.push_back("position_sequence_duplicate");else out.ordered.push_back(idx);
 }
 if(out.ordered.size()<=5)out.picked=out.ordered;else for(size_t j=0;j<5;++j)out.picked.push_back(out.ordered[j*(out.ordered.size()-1)/4]);
 return out;
}
struct Tagged {std::vector<PathGainWaypoint> poses;std::vector<int> part;};
inline Tagged tagged(const Input& in,const Sequence& path,size_t entry,size_t exit){
 Tagged out;double yaw=in.initialYaw;
 auto append=[&](const Eigen::Vector3d& p,double angle,int part){if(!out.poses.empty() && (out.poses.back().position-p).norm()<=1e-12 && angleDistance(out.poses.back().yaw,angle)<=1e-12)return;out.poses.push_back({p,angle});out.part.push_back(part);};
 auto rotate=[&](const Eigen::Vector3d& p,double target,int part){double d=std::remainder(target-yaw,2*std::acos(-1.));int n=std::ceil(std::abs(d)/.1);double begin=yaw;for(int i=1;i<=n;++i)append(p,begin+d*i/n,part);yaw=target;append(p,yaw,part);};
 append(position(in,path.front()),yaw,-1);
 for(size_t k=1;k<path.size();++k){size_t leg=k-1;auto a=position(in,path[k-1]),b=position(in,path[k]);double length=(b-a).norm();if(length<=1e-12)continue;
  double heading=std::atan2(b.y()-a.y(),b.x()-a.x());
  rotate(a,heading,leg<entry?-1:(leg<=exit?0:1));int n=std::ceil(length/.25);
  for(int i=1;i<=n;++i){Eigen::Vector3d p;for(int j=0;j<3;++j)p[j]=a[j]+(b[j]-a[j])*i/n;append(p,heading,leg<entry?-1:(leg<exit?0:1));}
 }
 rotate(position(in,path.back()),in.terminalYaw,exit==path.size()-1?0:1);return out;
}
inline std::vector<size_t> unchanged(const Tagged& t){std::vector<size_t> idx;for(size_t i=0;i<t.part.size();++i)if(t.part[i])idx.push_back(i);return idx;}
inline bool poseEqual(const PathGainWaypoint& a,const PathGainWaypoint& b){return (a.position-b.position).norm()<=1e-12 && angleDistance(a.yaw,b.yaw)<=1e-12;}
struct Metrics {uint64_t raw=0,unique=0;std::vector<uint64_t> marginal;std::vector<uint32_t> addresses;};
inline Metrics metrics(const std::vector<const CameraGainEvaluator::AddressSet*>& sets,CameraSharedBudget& budget){
 Metrics m;CameraGainEvaluator::AddressSet history;
 for(auto s:sets){budget.check();uint64_t added=0;m.raw+=s->size();for(auto id:*s){added+=history.insert(id).second;}m.marginal.push_back(added);}
 m.unique=history.size();m.addresses.assign(history.begin(),history.end());std::sort(m.addresses.begin(),m.addresses.end());budget.check();return m;
}
inline uint64_t difference(const std::vector<uint32_t>& a,const std::vector<uint32_t>& b){size_t i=0,j=0;uint64_t count=0;while(i<a.size()){while(j<b.size()&&b[j]<a[i])++j;if(j==b.size()||b[j]!=a[i])++count;++i;}return count;}
}
