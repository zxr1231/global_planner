#pragma once
#include <global_planner/cameraGainEvaluator.h>
#include <global_planner/routeSnapshot.h>
#include <global_planner/routeCandidates.h>
#include <memory>
namespace twoStageFixture {
using namespace globalPlanner;
using namespace routeSearch;
using namespace routeCandidates;
using Clock=std::chrono::steady_clock;
inline double elapsed(Clock::time_point p){return std::chrono::duration<double,std::milli>(Clock::now()-p).count();}
struct ContextError:std::runtime_error{using std::runtime_error::runtime_error;};
struct Input {
 mapManager::OccupancyMapSnapshot map;
 CameraGainConfig camera;
 PathGainVisibilityConfig legacy;
 std::vector<Graph::Position> positions;
 std::vector<Edge> edges;
 Sequence raw,reference;
 Id goal=0;double initialYaw=0,terminalYaw=0;
 MotionConfig motion;
 uint64_t expectedVersion=0;
 bool accepted=true;
};
struct Prepared {
 mapManager::OccupancyMapSnapshot map;
 std::unique_ptr<RouteSnapshot> view;
 std::unique_ptr<Graph> graph;
 Candidate reference;
};
inline Eigen::Vector3d position(const Input& in,Id id){const auto& p=in.positions.at(id);return {p[0],p[1],p[2]};}
inline bool safe(const Input& in,const RouteSnapshot& view,const Sequence& path){
 if(path.empty())return false;
 for(size_t i=0;i<path.size();++i){if(!view.free(position(in,path[i])))return false;if(i && !view.freeLine(position(in,path[i-1]),position(in,path[i])))return false;}return true;
}
inline std::unique_ptr<Prepared> prepareContext(const Input& in){
 if(in.expectedVersion!=in.map.version)throw ContextError("MAP_VERSION_MISMATCH");
 if(in.raw.empty() || in.reference.empty() || in.raw.front()!=0 || in.reference.front()!=0 || in.raw.back()!=in.goal || in.reference.back()!=in.goal)throw ContextError("REFERENCE_ENDPOINT_MISMATCH");
 auto p=std::make_unique<Prepared>();p->map=in.map;p->view=std::make_unique<RouteSnapshot>(p->map,in.legacy,std::vector<double>{0,3.141592653589793});
 if(!safe(in,*p->view,in.raw)||!safe(in,*p->view,in.reference))throw ContextError("REFERENCE_UNSAFE_IN_EXPORTED_MAP");
 std::vector<Edge> edges;for(const auto& e:in.edges)if(p->view->freeLine(position(in,e.first),position(in,e.second)))edges.push_back(e);
 p->graph=std::make_unique<Graph>(in.positions,edges);
 try{p->graph->pathCost(in.raw);}catch(const std::exception&){throw ContextError("REFERENCE_EDGE_MISSING");}
 p->reference.raw=in.raw;p->reference.simplified=in.reference;p->reference.poolIndex=0;
 p->reference.rawMotion=measure(*p->graph,in.raw,in.initialYaw,in.terminalYaw,in.motion);
 p->reference.simplifiedMotion=measure(*p->graph,in.reference,in.initialYaw,in.terminalYaw,in.motion);
 if(p->reference.simplifiedMotion.length<=1e-6 || p->reference.simplifiedMotion.time<=1e-6)throw ContextError("DEGENERATE_REFERENCE");
 return p;
}
inline std::vector<PathGainWaypoint> sample(const Input& in,const Sequence& path){
 if(path.empty())throw std::invalid_argument("empty route");
 std::vector<PathGainWaypoint> out;double yaw=in.initialYaw;
 auto append=[&](const Eigen::Vector3d& p,double angle){if(!out.empty() && (out.back().position-p).norm()<=1e-12 && angleDistance(out.back().yaw,angle)<=1e-12)return;out.push_back({p,angle});};
 auto rotate=[&](const Eigen::Vector3d& p,double target){double d=std::remainder(target-yaw,2*std::acos(-1.));int n=std::ceil(std::abs(d)/.1);double begin=yaw;for(int i=1;i<=n;++i)append(p,begin+d*i/n);yaw=target;append(p,yaw);};
 append(position(in,path.front()),yaw);
 for(size_t k=1;k<path.size();++k){auto a=position(in,path[k-1]),b=position(in,path[k]);double length=(b-a).norm();if(length<=1e-12)continue;double heading=std::atan2(b.y()-a.y(),b.x()-a.x());rotate(a,heading);int n=std::ceil(length/.25);for(int i=1;i<=n;++i){Eigen::Vector3d p;for(int j=0;j<3;++j)p[j]=a[j]+(b[j]-a[j])*i/n;append(p,heading);}}
 rotate(position(in,path.back()),in.terminalYaw);return out;
}
struct Pool {
 GoalPool pool;
 Selection selection;
 std::size_t pops=0;
 Stop stop=Stop::NONE;
 double milliseconds=0;
};
inline Pool generate(const Input& in,const Prepared& p,Mode mode,bool bounded,Clock::time_point scoreDeadline){
 auto begin=Clock::now();auto deadline=std::min(scoreDeadline,begin+std::chrono::milliseconds(50));
 Budget b;b.maxPops=10000;b.hasDeadline=bounded;b.deadline=deadline;
 Path reference{in.raw,p.graph->pathCost(in.raw)};
 auto result=yen(*p.graph,0,in.goal,6,b,&reference);
 auto proceed=[&]{return !bounded || Clock::now()<deadline;};
 Limits limits;limits.routesPerGoal=2;limits.globalRoutes=2;
 Pool out;out.pool=prepare(*p.graph,in.goal,result.paths,in.initialYaw,in.terminalYaw,in.motion,limits,[&](Id a,Id b){return p.view->freeLine(position(in,a),position(in,b));},proceed,&p.reference);
 out.selection=select({out.pool},mode,limits,proceed);out.pops=b.pops;out.stop=result.stop;out.milliseconds=elapsed(begin);return out;
}
struct Trial {
 std::string status="UNSET",reason;
 bool entered=false,refValid=false,complete=false,fallback=true,finalSafe=false,oracleMatch=true,motionValid=true;
 int cameraCalls=0;
 double totalMs=0,preparationMs=0,referenceMs=0,generationMs=0,alternativeMs=0,validationMs=0;
 CameraGainResult ref,alternative;
 Sequence alternativePath,selected;
 Pool supply;
 bool hasPool=false;
};
inline bool equalGain(const CameraGainResult& a,const CameraGainResult& b){return a.gain.valid==b.gain.valid && a.gain.rawGain==b.gain.rawGain && a.gain.uniqueAddresses==b.gain.uniqueAddresses && a.gain.marginalGains==b.gain.marginalGains;}
inline Trial trial(const Input& in,int mode,double budgetMs=500,uint64_t alternativeVisitLimit=std::numeric_limits<uint64_t>::max()){
 Trial r;auto begin=Clock::now();
 if(!in.accepted){r.status="SKIP_B_NEGATIVE_CONTROL";return r;}
 r.entered=true;
 try {
  if(budgetMs<=0){r.status="BUDGET_BEFORE_PREPARATION";r.totalMs=elapsed(begin);return r;}
  {
   auto p=prepareContext(in);r.refValid=true;
   CameraGainEvaluator evaluator(p->map,in.camera);
   auto refSamples=sample(in,in.reference);r.preparationMs=elapsed(begin);
   const double scoreLimit=std::max(0.,budgetMs-25.);
   auto score=[&](const std::vector<PathGainWaypoint>& poses,double time,uint64_t visits){CameraGainBudget b;b.maxMilliseconds=std::max(0.,scoreLimit-elapsed(begin));b.maxVoxelVisits=visits;++r.cameraCalls;return evaluator.evaluateSamples(poses,time,p->map.version,b);};
   r.ref=score(refSamples,p->reference.simplifiedMotion.time,std::numeric_limits<uint64_t>::max());r.referenceMs=r.ref.evaluationMs;
   if(!r.ref.gain.valid)r.status="REFERENCE_BUDGET_FALLBACK";
   else if(mode==0){r.complete=true;r.status="REFERENCE_COMPLETE";}
   else if(elapsed(begin)>=scoreLimit)r.status="GENERATION_BUDGET_FALLBACK";
   else {
    r.supply=generate(in,*p,mode==1?Mode::GENERIC_DISTANCE:Mode::GEOMETRIC_DIVERSE,true,begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double,std::milli>(scoreLimit)));r.hasPool=true;r.generationMs=r.supply.milliseconds;
    if(r.generationMs>50 || elapsed(begin)>=scoreLimit)r.status="GENERATION_BUDGET_FALLBACK";
    else if(r.supply.selection.routes.size()!=2)r.status="SUPPLY_DEFICIT";
    else {
     const auto& alt=r.supply.pool.records.at(r.supply.selection.routes[1].poolIndex);r.alternativePath=alt.simplified;
     std::vector<std::string> errors;Limits limits;motionRejections(alt.rawMotion,p->reference.rawMotion,limits,"raw",errors);motionRejections(alt.simplifiedMotion,p->reference.simplifiedMotion,limits,"final",errors);
     r.motionValid=errors.empty() && !sameGeometry(*p->graph,in.reference,alt.simplified);
     if(!r.motionValid)throw std::runtime_error("invalid/duplicate alternative escaped constraints");
     auto poses=sample(in,alt.simplified);r.alternative=score(poses,alt.simplifiedMotion.time,alternativeVisitLimit);r.alternativeMs=r.alternative.evaluationMs;
     if(!r.alternative.gain.valid){r.status="ALTERNATIVE_BUDGET_FALLBACK";if(r.alternative.gain.rawGain || r.alternative.gain.uniqueGain || !r.alternative.gain.marginalGains.empty() || !r.alternative.gain.uniqueAddresses.empty())throw std::runtime_error("partial gain escaped");}
     else {r.complete=true;r.status="TWO_COMPLETE";}
    }
   }
   r.selected=in.reference;
   if(r.complete && mode && r.alternative.gain.uniqueUtility>r.ref.gain.uniqueUtility+1e-9)r.selected=r.alternativePath;
   auto check=Clock::now();r.finalSafe=safe(in,*p->view,r.selected);
   if(!r.finalSafe){r.complete=false;r.status="FINAL_UNSAFE";r.selected.clear();}
   r.validationMs=elapsed(check);
  } // evaluator/map/graph/scratch cleanup is included in total wall time
 }catch(const ContextError& e){r.status="REPLAY_CONTEXT_INVALID";r.reason=e.what();r.selected.clear();r.complete=false;}
 r.totalMs=elapsed(begin);
 if(r.totalMs>budgetMs && r.complete){r.complete=false;r.status="TOTAL_BUDGET_FALLBACK";r.selected=in.reference;/* reference already checked on this immutable map */}
 r.fallback=!r.complete;
 return r;
}
inline bool verify(const Input& in,const Trial& r){
 if(!r.entered || !r.refValid)return true;
 CameraGainEvaluator e(in.map,in.camera);auto p=prepareContext(in);
 if(r.ref.gain.valid && !equalGain(r.ref,e.evaluateSamples(sample(in,in.reference),p->reference.simplifiedMotion.time,in.map.version)))return false;
 if(r.alternative.gain.valid){auto m=measure(*p->graph,r.alternativePath,in.initialYaw,in.terminalYaw,in.motion);if(!equalGain(r.alternative,e.evaluateSamples(sample(in,r.alternativePath),m.time,in.map.version)))return false;}
 return true;
}
}
