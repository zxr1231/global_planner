#include "snapshotFixtureIO.h"
#include "corridorFixture.h"
using namespace corridorFixture;
Json::Value ids(const Sequence& s){Json::Value a(Json::arrayValue);for(auto n:s)a.append(Json::UInt64(n));return a;}
Json::Value posesJson(const std::vector<PathGainWaypoint>& poses){Json::Value a(Json::arrayValue);for(auto p:poses){Json::Value q;q["position"]=Json::Value(Json::arrayValue);for(int k=0;k<3;++k)q["position"].append(p.position[k]);q["yaw"]=p.yaw;a.append(q);}return a;}
Json::Value motionJson(const Motion& m){Json::Value j;j["length"]=m.length;j["time"]=m.time;j["yaw"]=m.yaw;return j;}
Json::Value metricsJson(const Metrics& m){Json::Value j;j["raw"]=Json::UInt64(m.raw);j["unique"]=Json::UInt64(m.unique);j["marginal"]=Json::Value(Json::arrayValue);for(auto n:m.marginal)j["marginal"].append(Json::UInt64(n));j["addresses"]=Json::Value(Json::arrayValue);for(auto n:m.addresses)j["addresses"].append(Json::UInt(n));return j;}
Input loadInput(const std::string& dir,const Json::Value& req){
 auto manifest=loadJson(dir+"/manifest.json");if(!manifest["complete"].asBool()||manifest["schema"].asString()!="cerlab-r1-snapshot-v1")throw ContextError("manifest invalid");
 auto mp=dir+"/"+manifest["map_file"].asString(),pp=dir+"/"+manifest["planner_file"].asString();
 if(hex64(fnv1a64(mp))!=manifest["map_fnv1a64"].asString()||hex64(fnv1a64(pp))!=manifest["planner_fnv1a64"].asString())throw ContextError("snapshot hash mismatch");
 auto plan=loadJson(pp);Input in;in.map=loadMap(mp);in.expectedVersion=req["map_version"].asUInt64();
 if(in.map.version!=in.expectedVersion||in.map.version!=manifest["map_version"].asUInt64()||in.map.version!=plan["map_version"].asUInt64())throw ContextError("MAP_VERSION_MISMATCH");
 in.accepted=req["accepted"].asBool();in.goal=req["goal"].asUInt64();in.initialYaw=number(req["start_yaw"]);in.terminalYaw=number(req["terminal_yaw"]);
 auto c=req["camera"];in.camera.fx=number(c["fx"]);in.camera.fy=number(c["fy"]);in.camera.cx=number(c["cx"]);in.camera.cy=number(c["cy"]);in.camera.cols=c["image_cols"].asInt();in.camera.rows=c["image_rows"].asInt();in.camera.margin=c["filter_margin"].asInt();in.camera.pixelSkip=c["evaluation_pixel_skip"].asInt();in.camera.range=number(c["raycast_max_length"]);
 for(int r=0;r<4;++r)for(int k=0;k<4;++k)in.camera.bodyToCamera(r,k)=number(c["body_to_camera"][4*r+k]);
 for(int k=0;k<3;++k){in.camera.planningMin[k]=in.legacy.planningMin[k]=number(c["planning_min"][k]);in.camera.planningMax[k]=in.legacy.planningMax[k]=number(c["planning_max"][k]);}
 in.legacy.horizontalFov=number(plan["sensor_model"]["horizontal_fov"]);in.legacy.verticalFov=number(plan["sensor_model"]["vertical_fov"]);in.legacy.dmax=number(plan["sensor_model"]["dmax"]);
 in.motion.speed=number(plan["motion_model"]["velocity"]);in.motion.angularSpeed=number(plan["motion_model"]["angular_velocity"]);in.motion.yawPenalty=number(plan["motion_model"]["yaw_penalty_weight"]);
 for(auto p:req["nodes"])in.positions.push_back({number(p[0]),number(p[1]),number(p[2])});
 for(auto e:req["edges"])in.edges.push_back({e[0].asUInt64(),e[1].asUInt64()});
 for(auto n:req["raw"])in.raw.push_back(n.asUInt64());
 for(auto n:req["reference"])in.reference.push_back(n.asUInt64());
 return in;
}
std::vector<PathGainWaypoint> parsePoses(const Json::Value& j){std::vector<PathGainWaypoint> out;for(auto p:j){PathGainWaypoint q;for(int k=0;k<3;++k)q.position[k]=number(p["position"][k]);q.yaw=number(p["yaw"]);out.push_back(q);}return out;}
Json::Value poolOutput(const Input& in){auto begin=Clock::now();auto pool=generate(in);Json::Value out;out["generation_component_ms"]=elapsed(begin);out["status"]="CONTEXT_VALID";out["reference_motion"]=motionJson(pool.reference);out["possible_pairs"]=Json::UInt64(pool.possible);out["truncated"]=pool.truncated;out["camera_calls"]=0;out["records"]=Json::Value(Json::arrayValue);out["ordered"]=ids(pool.ordered);out["pool"]=ids(pool.picked);out["reference_samples"]=posesJson(sample(in,in.reference));
 for(size_t i=0;i<pool.records.size();++i){auto& r=pool.records[i];Json::Value j;j["record_id"]=Json::UInt64(i);j["e"]=Json::UInt64(r.e);j["u"]=Json::UInt64(r.u);j["v"]=Json::UInt64(r.v);j["exit"]=Json::UInt64(r.exit);j["path"]=ids(r.path);j["measured"]=r.measured;j["reasons"]=Json::Value(Json::arrayValue);for(auto reason:r.reasons)j["reasons"].append(reason);if(r.measured)j["motion"]=motionJson(r.motion);
  if(std::find(pool.picked.begin(),pool.picked.end(),i)!=pool.picked.end()){
   auto ref=tagged(in,in.reference,r.e,r.e+1),cand=tagged(in,r.path,r.e,r.exit);auto a=unchanged(ref),b=unchanged(cand);
   if(a.size()!=b.size())throw ContextError("UNCHANGED_POSE_COUNT");
   for(size_t k=0;k<a.size();++k){if(!poseEqual(ref.poses[a[k]],cand.poses[b[k]]))throw ContextError("UNCHANGED_POSE_MISMATCH");cand.poses[b[k]]=ref.poses[a[k]];}
   auto oracle=sample(in,r.path);if(oracle.size()!=cand.poses.size())throw ContextError("SAMPLER_COUNT_MISMATCH");for(size_t k=0;k<oracle.size();++k)if(!poseEqual(oracle[k],cand.poses[k]))throw ContextError("SAMPLER_POSE_MISMATCH");
   j["samples"]=posesJson(cand.poses);j["reference_parts"]=Json::Value(Json::arrayValue);for(auto p:ref.part)j["reference_parts"].append(p);
   j["candidate_parts"]=Json::Value(Json::arrayValue);for(auto p:cand.part)j["candidate_parts"].append(p);
  }out["records"].append(j);
 }return out;
}
struct Evaluated {size_t record=0;Metrics m;uint64_t added=0,lost=0,changedPoses=0;double time=0,componentMs=0;};
Json::Value composeOutput(const Input& in,const Json::Value& pool){
 auto begin=Clock::now();CameraGainEvaluator eval(in.map,in.camera);double prep=elapsed(begin);
 fixtureAllocation::Ledger ledger;Metrics reference;std::vector<Evaluated> rows;CameraSharedBudget budget;Json::Value out;
 {
  fixtureAllocation::Scope guard(ledger);
  auto rp=parsePoses(pool["reference_samples"]);auto ref=eval.evaluatePoseSets(rp,in.expectedVersion,budget);
  if(!ref.valid)throw ContextError(ref.failureReason);
  std::vector<const CameraGainEvaluator::AddressSet*> ptr;
  for(auto& s:ref.poseSets)ptr.push_back(&s);
  reference=metrics(ptr,budget);
  for(auto idx:pool["pool"]){size_t id=idx.asUInt64();auto rec=pool["records"][Json::ArrayIndex(id)];auto cp=parsePoses(rec["samples"]);std::vector<PathGainWaypoint> changed;std::vector<size_t> same;
   for(size_t k=0;k<rp.size();++k)if(rec["reference_parts"][Json::ArrayIndex(k)].asInt())same.push_back(k);
   for(size_t k=0;k<cp.size();++k)if(!rec["candidate_parts"][Json::ArrayIndex(k)].asInt())changed.push_back(cp[k]);
   auto cb=Clock::now();auto local=eval.evaluatePoseSets(changed,in.expectedVersion,budget);if(!local.valid)throw ContextError(local.failureReason);
   ptr.clear();size_t si=0,ci=0;for(size_t k=0;k<cp.size();++k){if(rec["candidate_parts"][Json::ArrayIndex(k)].asInt()){if(si>=same.size()||!poseEqual(cp[k],rp[same[si]]))throw ContextError("COMPOSE_UNCHANGED_MISMATCH");ptr.push_back(&ref.poseSets[same[si++]]);}else ptr.push_back(&local.poseSets[ci++]);}
   if(si!=same.size()||ci!=local.poseSets.size())throw ContextError("COMPOSE_INDEX_MISMATCH");
   Evaluated row;row.record=id;row.time=number(rec["motion"]["time"]);row.m=metrics(ptr,budget);row.added=difference(row.m.addresses,reference.addresses);row.lost=difference(reference.addresses,row.m.addresses);row.changedPoses=changed.size();row.componentMs=elapsed(cb);rows.push_back(std::move(row));
  }budget.check();
 }
 out["status"]="COMPLETE";out["valid"]=true;out["reference"]=metricsJson(reference);out["rows"]=Json::Value(Json::arrayValue);
 for(auto& row:rows){Json::Value j;j["record_id"]=Json::UInt64(row.record);j["metrics"]=metricsJson(row.m);j["new"]=Json::UInt64(row.added);j["lost"]=Json::UInt64(row.lost);j["time"]=row.time;j["utility"]=row.m.unique/row.time;j["changed_pose_count"]=Json::UInt64(row.changedPoses);j["component_ms"]=row.componentMs;out["rows"].append(j);}
 out["preparation_component_ms"]=prep;out["shared_camera_pose_queries"]=Json::UInt64(budget.samples);out["shared_voxel_visits"]=Json::UInt64(budget.voxelVisits);out["visibility_requested_peak_bytes"]=Json::UInt64(ledger.peak);out["allocation_limit_rejected"]=ledger.rejected;
 out["input_map_bytes"]=Json::UInt64(in.map.occupancy.size()*sizeof(double)+(in.map.inflated.capacity()+7)/8);out["phase_ms_before_serialization"]=elapsed(begin);out["map_version"]=Json::UInt64(in.map.version);return out;
}
Json::Value oracleOutput(const Input& in,const Json::Value& pool){
 auto begin=Clock::now();CameraGainEvaluator eval(in.map,in.camera);fixtureAllocation::Ledger ledger;std::vector<Evaluated> rows;CameraSharedBudget budget;
 {
  fixtureAllocation::Scope guard(ledger);auto query=[&](const std::vector<PathGainWaypoint>& poses,size_t id,double time){auto result=eval.evaluatePoseSets(poses,in.expectedVersion,budget);if(!result.valid)throw ContextError(result.failureReason);std::vector<const CameraGainEvaluator::AddressSet*> p;for(auto& s:result.poseSets)p.push_back(&s);Evaluated row;row.record=id;row.time=time;row.m=metrics(p,budget);rows.push_back(std::move(row));};
  query(sample(in,in.reference),std::numeric_limits<size_t>::max(),number(pool["reference_motion"]["time"]));
  for(auto id:pool["pool"]){auto r=pool["records"][Json::ArrayIndex(id.asUInt64())];Sequence path;for(auto n:r["path"])path.push_back(n.asUInt64());query(sample(in,path),id.asUInt64(),number(r["motion"]["time"]));}
 }
 Json::Value out;out["status"]="COMPLETE";out["rows"]=Json::Value(Json::arrayValue);for(auto& r:rows){Json::Value j;j["record_id"]=r.record==std::numeric_limits<size_t>::max()?Json::Value(-1):Json::Value(Json::UInt64(r.record));j["metrics"]=metricsJson(r.m);out["rows"].append(j);}out["component_ms"]=elapsed(begin);out["visibility_requested_peak_bytes"]=Json::UInt64(ledger.peak);out["shared_camera_pose_queries"]=Json::UInt64(budget.samples);return out;
}
int main(int argc,char** argv){try{
 if(argc!=4&&argc!=5)throw std::invalid_argument("usage: corridor_fixture_runner pool|compose|oracle SNAPSHOT REQUEST [POOL]");
 auto request=loadJson(argv[3]);Json::Value out;
 if(!request["accepted"].asBool()){out["status"]="SKIP_NEGATIVE_CONTROL";out["camera_calls"]=0;out["pairs_enumerated"]=0;}
 else{auto in=loadInput(argv[2],request);std::string mode=argv[1];if(mode=="pool")out=poolOutput(in);else{if(argc!=5)throw std::invalid_argument("pool input required");auto pool=loadJson(argv[4]);if(mode=="compose")out=composeOutput(in,pool);else if(mode=="oracle")out=oracleOutput(in,pool);else throw std::invalid_argument("unknown mode");}}
 Json::StreamWriterBuilder w;w["indentation"]="";std::cout<<Json::writeString(w,out)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
