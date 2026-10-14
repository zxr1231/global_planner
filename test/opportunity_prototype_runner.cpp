#include "snapshotFixtureIO.h"
#include "twoStageFixture.h"
#include "fixtureAllocation.h"
#include <global_planner/opportunityRoutes.h>
namespace o=globalPlanner::opportunityRoutes;
using namespace twoStageFixture;
struct Result {
 std::string status="UNSET",reason;bool contextValid=false,complete=false,safe=false,oracle=true;
 double totalMs=0,preparationMs=0,referenceMs=0,generationMs=0,scoringMs=0;
 size_t exactAttempted=0,exactCompleted=0,modelBytes=0,coarseModelBytes=0;
 size_t coarseSamples=0;uint64_t coarseVisits=0;
 CameraGainResult reference;std::vector<CameraGainResult> exact;o::Result generation;Sequence selected;
};
Result trial(const Input& in,int mode,bool bounded){
 Result r;auto start=Clock::now();const auto scoreDeadline=bounded?start+std::chrono::milliseconds(475):Clock::time_point::max();
 try {
  {auto p=prepareContext(in);r.contextValid=r.safe=true;r.selected=in.reference;
   CameraGainEvaluator full(in.map,in.camera);r.modelBytes=full.allocatedCapacityBytes();r.preparationMs=elapsed(start);
   auto score=[&](const Sequence& path,double time){CameraGainBudget b;if(bounded)b.maxMilliseconds=std::max(0.,std::chrono::duration<double,std::milli>(scoreDeadline-Clock::now()).count());++r.exactAttempted;auto result=full.evaluateSamples(o::samplePath(*p->graph,path,in.initialYaw,in.terminalYaw),time,in.map.version,b);if(result.gain.valid)++r.exactCompleted;return result;};
   r.reference=score(in.reference,p->reference.simplifiedMotion.time);r.referenceMs=elapsed(start)-r.preparationMs;
   if(!r.reference.gain.valid){r.status="REFERENCE_BUDGET_FALLBACK";r.reason=r.reference.failureReason;}
   else if(mode==0){r.complete=true;r.status="REFERENCE_COMPLETE";}
   else {
    const auto generationStart=Clock::now();auto generationDeadline=bounded?std::min(scoreDeadline,generationStart+std::chrono::milliseconds(50)):Clock::time_point::max();
    std::unique_ptr<CameraGainEvaluator> coarse;CameraSharedBudget coarseBudget;coarseBudget.deadline=generationDeadline;coarseBudget.maxSamples=10000;coarseBudget.maxVoxelVisits=100000000;
    auto observe=[&](const Sequence& path){
     coarseBudget.check();if(!coarse){auto c=in.camera;c.pixelSkip=32;coarse=std::make_unique<CameraGainEvaluator>(in.map,c);r.coarseModelBytes=coarse->allocatedCapacityBytes();}
     auto poses=o::samplePath(*p->graph,path,in.initialYaw,in.terminalYaw,1.,.35);auto v=coarse->evaluatePoseSets(poses,in.map.version,coarseBudget,0);if(!v.valid)throw std::runtime_error(v.failureReason);
     CameraGainEvaluator::AddressSet unique;for(const auto& s:v.poseSets){coarseBudget.check();unique.insert(s.begin(),s.end());}
     std::vector<uint32_t> out(unique.begin(),unique.end());std::sort(out.begin(),out.end());coarseBudget.check();return out;
    };
    try {r.generation=o::generate(*p->graph,in.reference,in.initialYaw,in.terminalYaw,in.motion,mode==1?o::Mode::MOTION:(mode==2?o::Mode::DIVERSE:o::Mode::OBSERVATION),[&](Id a,Id b){return p->view->freeLine(position(in,a),position(in,b));},observe,[&]{return Clock::now()<generationDeadline;});}
    catch(const std::exception& e){r.reason=e.what();r.status="GENERATION_RESOURCE_FALLBACK";}
    r.generationMs=elapsed(generationStart);r.coarseSamples=coarseBudget.samples;r.coarseVisits=coarseBudget.voxelVisits;
    if(r.status=="UNSET"&&bounded&&(r.generationMs>50||Clock::now()>=scoreDeadline))r.status="GENERATION_BUDGET_FALLBACK";
    if(r.status=="UNSET"){
     auto scoringStart=Clock::now();r.complete=true;size_t best=0;double bestU=r.reference.gain.uniqueUtility;
     for(const auto& route:r.generation.routes){auto exact=score(route.path,route.motion.time);r.exact.push_back(std::move(exact));if(!r.exact.back().gain.valid){r.complete=false;r.reason=r.exact.back().failureReason;break;}if(r.exact.back().gain.uniqueUtility>bestU+1e-9){bestU=r.exact.back().gain.uniqueUtility;best=r.exact.size();}}
     r.scoringMs=elapsed(scoringStart);r.status=r.complete?"COMPLETE":"ALTERNATIVE_BUDGET_FALLBACK";
     if(r.complete&&best)r.selected=r.generation.routes[best-1].path;
    }
   }
   r.safe=safe(in,*p->view,r.selected);if(!r.safe){r.complete=false;r.selected.clear();r.status="FINAL_UNSAFE";}
  }
 }catch(const ContextError& e){r.status="CONTEXT_INVALID";r.reason=e.what();r.selected.clear();r.safe=false;r.complete=false;}
 catch(const std::exception& e){r.status="RESOURCE_OR_TOOL_FALLBACK";r.reason=e.what();r.complete=false;if(r.contextValid){r.selected=in.reference;}else r.selected.clear();}
 r.totalMs=elapsed(start);if(bounded&&r.totalMs>500){r.complete=false;r.status="TOTAL_BUDGET_FALLBACK";if(r.contextValid&&r.safe)r.selected=in.reference;else r.selected.clear();}
 return r;
}
Json::Value ids(const Sequence& path){Json::Value j(Json::arrayValue);for(auto id:path)j.append(Json::UInt64(id));return j;}
Json::Value gain(const CameraGainResult& r){Json::Value j;j["valid"]=r.gain.valid;j["raw_gain"]=Json::UInt64(r.gain.rawGain);j["unique_gain"]=Json::UInt64(r.gain.uniqueGain);j["utility"]=r.gain.uniqueUtility;j["estimated_time"]=r.gain.estimatedTime;j["reason"]=r.failureReason;j["unique_addresses"]=ids(Sequence(r.gain.uniqueAddresses.begin(),r.gain.uniqueAddresses.end()));return j;}
Json::Value serialize(const Result& r){Json::Value j;j["status"]=r.status;j["reason"]=r.reason;j["context_valid"]=r.contextValid;j["complete"]=r.complete;j["safe"]=r.safe;j["oracle_match"]=r.oracle;j["total_ms"]=r.totalMs;j["preparation_ms"]=r.preparationMs;j["reference_ms"]=r.referenceMs;j["generation_ms"]=r.generationMs;j["scoring_ms"]=r.scoringMs;j["exact_attempted"]=Json::UInt64(r.exactAttempted);j["exact_completed"]=Json::UInt64(r.exactCompleted);j["model_capacity_bytes"]=Json::UInt64(r.modelBytes);j["coarse_model_capacity_bytes"]=Json::UInt64(r.coarseModelBytes);j["coarse_samples"]=Json::UInt64(r.coarseSamples);j["coarse_voxel_visits"]=Json::UInt64(r.coarseVisits);j["selected"]=ids(r.selected);j["reference"]=gain(r.reference);
 j["generation"]["actions"]=Json::UInt64(r.generation.actions);j["generation"]["first_feasible"]=Json::UInt64(r.generation.firstFeasible);j["generation"]["second_feasible"]=Json::UInt64(r.generation.secondFeasible);j["generation"]["coarse_queries"]=Json::UInt64(r.generation.coarseQueries);j["generation"]["operation_cutoff"]=r.generation.operationCutoff;j["generation"]["beam"]=Json::Value(Json::arrayValue);for(const auto& b:r.generation.beam)j["generation"]["beam"].append(ids(b));j["routes"]=Json::Value(Json::arrayValue);
 for(size_t i=0;i<r.generation.routes.size();++i){const auto& a=r.generation.routes[i];Json::Value row;row["path"]=ids(a.path);row["layer"]=Json::UInt64(a.layer);row["segment"]=Json::UInt64(a.segment);row["length"]=a.motion.length;row["time"]=a.motion.time;row["yaw"]=a.motion.yaw;row["proxy_unique_gain"]=Json::UInt64(a.observed.size());row["proxy_new"]=Json::UInt64(a.newVoxels);row["proxy_lost"]=Json::UInt64(a.lostVoxels);row["proxy_utility"]=a.proxyUtility;if(i<r.exact.size())row["exact"]=gain(r.exact[i]);j["routes"].append(row);}return j;
}
Input input(const std::string& dir,const Json::Value& request){
 auto manifest=loadJson(dir+"/manifest.json");if(!manifest["complete"].asBool()||manifest["schema"].asString()!="cerlab-r1-snapshot-v1")throw std::runtime_error("manifest invalid");
 auto mapfile=dir+"/"+manifest["map_file"].asString(),plannerfile=dir+"/"+manifest["planner_file"].asString();if(hex64(fnv1a64(mapfile))!=manifest["map_fnv1a64"].asString()||hex64(fnv1a64(plannerfile))!=manifest["planner_fnv1a64"].asString())throw std::runtime_error("source hash mismatch");
 Input in;in.map=loadMap(mapfile);in.expectedVersion=request["map_version"].asUInt64();in.accepted=request["accepted"].asBool();in.goal=request["goal"].asUInt64();in.initialYaw=number(request["start_yaw"]);in.terminalYaw=number(request["terminal_yaw"]);auto planner=loadJson(plannerfile),c=request["camera"];
 if(in.map.version!=manifest["map_version"].asUInt64()||in.map.version!=planner["map_version"].asUInt64())throw std::runtime_error("source version mismatch");
 in.camera.fx=number(c["fx"]);in.camera.fy=number(c["fy"]);in.camera.cx=number(c["cx"]);in.camera.cy=number(c["cy"]);in.camera.cols=c["image_cols"].asInt();in.camera.rows=c["image_rows"].asInt();in.camera.margin=c["filter_margin"].asInt();in.camera.pixelSkip=c["evaluation_pixel_skip"].asInt();in.camera.range=number(c["raycast_max_length"]);for(int r=0;r<4;++r)for(int k=0;k<4;++k)in.camera.bodyToCamera(r,k)=number(c["body_to_camera"][4*r+k]);
 in.legacy.horizontalFov=number(planner["sensor_model"]["horizontal_fov"]);in.legacy.verticalFov=number(planner["sensor_model"]["vertical_fov"]);in.legacy.dmax=number(planner["sensor_model"]["dmax"]);for(int a=0;a<3;++a){in.camera.planningMin[a]=in.legacy.planningMin[a]=number(c["planning_min"][a]);in.camera.planningMax[a]=in.legacy.planningMax[a]=number(c["planning_max"][a]);}
 in.motion.speed=number(planner["motion_model"]["velocity"]);in.motion.angularSpeed=number(planner["motion_model"]["angular_velocity"]);in.motion.yawPenalty=number(planner["motion_model"]["yaw_penalty_weight"]);for(auto p:request["nodes"])in.positions.push_back({number(p[0]),number(p[1]),number(p[2])});for(auto e:request["edges"])in.edges.push_back({e[0].asUInt64(),e[1].asUInt64()});for(auto n:request["raw"])in.raw.push_back(n.asUInt64());for(auto n:request["reference"])in.reference.push_back(n.asUInt64());return in;
}
int main(int argc,char** argv){try{
 if(argc!=5)throw std::invalid_argument("usage: opportunity_prototype_runner SNAPSHOT REQUEST MODE mechanism|budget");
 int mode=std::stoi(argv[3]);if(mode<0||mode>3)throw std::invalid_argument("mode");bool bounded=std::string(argv[4])=="budget";if(!bounded&&std::string(argv[4])!="mechanism")throw std::invalid_argument("regime");auto request=loadJson(argv[2]);Json::Value out;out["mode"]=mode;out["regime"]=argv[4];out["trials"]=Json::Value(Json::arrayValue);
 if(!request["accepted"].asBool()){out["status"]="SKIP_NEGATIVE_CONTROL";out["camera_queries"]=0;}
 else {auto io=Clock::now();auto in=input(argv[1],request);out["input_io_ms"]=elapsed(io);out["status"]="EXECUTED";
  if(bounded){fixtureAllocation::Ledger warmLedger;Result warm;{fixtureAllocation::Scope scope(warmLedger);warm=trial(in,mode,true);}out["warmup"]["status"]=warm.status;out["warmup"]["complete"]=warm.complete;out["warmup"]["total_ms"]=warm.totalMs;}
  for(int repeat=0;repeat<(bounded?3:1);++repeat){fixtureAllocation::Ledger ledger;Result r;{fixtureAllocation::Scope scope(ledger);r=trial(in,mode,bounded);}
   if(!bounded&&r.contextValid&&r.reference.gain.valid){CameraGainEvaluator oracle(in.map,in.camera);auto p=prepareContext(in);auto ref=oracle.evaluateSamples(sample(in,in.reference),p->reference.simplifiedMotion.time,in.map.version);r.oracle=equalGain(ref,r.reference);
    if(r.complete&&r.selected!=in.reference){auto pos=std::find_if(r.generation.routes.begin(),r.generation.routes.end(),[&](const o::Record& row){return row.path==r.selected;});size_t i=pos-r.generation.routes.begin();auto check=oracle.evaluateSamples(sample(in,r.selected),pos->motion.time,in.map.version);r.oracle=r.oracle&&equalGain(check,r.exact.at(i));}
    if(!r.oracle)throw std::runtime_error("selected full score differs from independent oracle");
   }auto j=serialize(r);j["repeat"]=repeat;j["scratch_peak_bytes"]=Json::UInt64(ledger.peak);j["allocation_limit_rejected"]=ledger.rejected;j["generation_counters_available"]=r.status!="GENERATION_RESOURCE_FALLBACK";out["trials"].append(j);
  }
 }Json::StreamWriterBuilder w;w["indentation"]="";std::cout<<Json::writeString(w,out)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
