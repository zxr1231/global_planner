#include "snapshotFixtureIO.h"
#include "twoStageFixture.h"
using namespace twoStageFixture;
Json::Value ids(const Sequence& s){Json::Value j(Json::arrayValue);for(auto n:s)j.append(Json::UInt64(n));return j;}
Json::Value gain(const CameraGainResult& r){Json::Value j;j["valid"]=r.gain.valid;j["raw"]=Json::UInt64(r.gain.rawGain);j["unique"]=Json::UInt64(r.gain.uniqueGain);j["time"]=r.gain.estimatedTime;j["utility"]=r.gain.uniqueUtility;j["samples"]=r.gain.sampleCount;j["map_version"]=Json::UInt64(r.gain.mapVersion);j["reason"]=r.failureReason;j["marginals"]=Json::Value(Json::arrayValue);for(auto n:r.gain.marginalGains)j["marginals"].append(Json::UInt64(n));return j;}
Json::Value poolJson(const Pool& p){Json::Value j;j["raw_count"]=Json::UInt64(p.pool.counts.rawGenerated);j["feasible_count"]=Json::UInt64(p.pool.counts.motionFeasible);j["distinct_count"]=Json::UInt64(p.pool.counts.postShortcutUnique);j["pops"]=Json::UInt64(p.pops);j["stop"]=int(p.stop);j["elapsed_ms"]=p.milliseconds;j["records"]=Json::Value(Json::arrayValue);for(const auto& r:p.pool.records){Json::Value x;x["pool_index"]=Json::UInt64(r.poolIndex);x["raw"]=ids(r.raw);x["final"]=ids(r.simplified);x["reasons"]=Json::Value(Json::arrayValue);for(const auto& reason:r.reasons)x["reasons"].append(reason);j["records"].append(x);}return j;}
Json::Value trialJson(const Trial& r,int mode,int repeat){Json::Value j;j["mode"]=mode;j["repeat"]=repeat;j["status"]=r.status;j["reason"]=r.reason;j["entered"]=r.entered;j["reference_valid"]=r.refValid;j["complete"]=r.complete;j["fallback"]=r.fallback;j["final_safe"]=r.finalSafe;j["oracle_match"]=r.oracleMatch;j["motion_valid"]=r.motionValid;j["camera_calls"]=r.cameraCalls;j["total_ms"]=r.totalMs;j["preparation_ms"]=r.preparationMs;j["reference_ms"]=r.referenceMs;j["generation_ms"]=r.generationMs;j["alternative_ms"]=r.alternativeMs;j["validation_ms"]=r.validationMs;j["reference"]=gain(r.ref);j["alternative"]=gain(r.alternative);j["alternative_path"]=ids(r.alternativePath);j["selected"]=ids(r.selected);if(r.hasPool)j["pool"]=poolJson(r.supply);return j;}
int main(int argc,char** argv){try{
 if(argc!=3)throw std::invalid_argument("usage: two_stage_fixture_runner SNAPSHOT_DIR REQUEST");
 const std::string dir=argv[1];auto request=loadJson(argv[2]);auto manifest=loadJson(dir+"/manifest.json");
 if(!manifest["complete"].asBool() || manifest["schema"].asString()!="cerlab-r1-snapshot-v1")throw std::runtime_error("manifest invalid");
 const auto mapfile=dir+"/"+manifest["map_file"].asString(),plannerfile=dir+"/"+manifest["planner_file"].asString();
 if(hex64(fnv1a64(mapfile))!=manifest["map_fnv1a64"].asString()||hex64(fnv1a64(plannerfile))!=manifest["planner_fnv1a64"].asString())throw std::runtime_error("frozen hash mismatch");
 Input in;in.map=loadMap(mapfile);in.expectedVersion=request["map_version"].asUInt64();in.accepted=request["accepted"].asBool();in.goal=request["goal"].asUInt64();in.initialYaw=number(request["start_yaw"]);in.terminalYaw=number(request["terminal_yaw"]);
 const auto planner=loadJson(plannerfile),c=request["camera"];
 if(planner["map_version"].asUInt64()!=in.map.version || manifest["map_version"].asUInt64()!=in.map.version)throw std::runtime_error("source version mismatch");
 in.camera.fx=number(c["fx"]);in.camera.fy=number(c["fy"]);in.camera.cx=number(c["cx"]);in.camera.cy=number(c["cy"]);in.camera.cols=c["image_cols"].asInt();in.camera.rows=c["image_rows"].asInt();in.camera.margin=c["filter_margin"].asInt();in.camera.pixelSkip=c["evaluation_pixel_skip"].asInt();in.camera.range=number(c["raycast_max_length"]);
 for(int r=0;r<4;++r)for(int k=0;k<4;++k)in.camera.bodyToCamera(r,k)=number(c["body_to_camera"][4*r+k]);
 in.legacy.horizontalFov=number(planner["sensor_model"]["horizontal_fov"]);in.legacy.verticalFov=number(planner["sensor_model"]["vertical_fov"]);in.legacy.dmax=number(planner["sensor_model"]["dmax"]);
 for(int a=0;a<3;++a){in.camera.planningMin[a]=in.legacy.planningMin[a]=number(c["planning_min"][a]);in.camera.planningMax[a]=in.legacy.planningMax[a]=number(c["planning_max"][a]);}
 in.motion.speed=number(planner["motion_model"]["velocity"]);in.motion.angularSpeed=number(planner["motion_model"]["angular_velocity"]);in.motion.yawPenalty=number(planner["motion_model"]["yaw_penalty_weight"]);
 for(const auto& p:request["nodes"])in.positions.push_back({number(p[0]),number(p[1]),number(p[2])});
 for(const auto& e:request["edges"])in.edges.push_back({e[0].asUInt64(),e[1].asUInt64()});
 for(const auto& n:request["raw"])in.raw.push_back(n.asUInt64());
 for(const auto& n:request["reference"])in.reference.push_back(n.asUInt64());
 Json::Value output;output["sequence"]=request["sequence"];output["trials"]=Json::Value(Json::arrayValue);
 if(!in.accepted){output["status"]="SKIP_B_NEGATIVE_CONTROL";output["camera_calls"]=0;output["selected"]=Json::Value(Json::arrayValue);}
 else {
  try {
   auto p=prepareContext(in);auto generic=generate(in,*p,Mode::GENERIC_DISTANCE,false,Clock::time_point::max());output["operation_supply"]=poolJson(generic);output["status"]="CONTEXT_VALID";
   output["reference_samples"]=Json::Value(Json::arrayValue);for(const auto& pose:sample(in,in.reference)){Json::Value row(Json::arrayValue);for(int a=0;a<3;++a)row.append(pose.position[a]);row.append(pose.yaw);output["reference_samples"].append(row);}
   p.reset();
   for(const auto& m:request["mode_order"]){auto warm=trial(in,m.asInt());(void)warm;}
   struct Measured {int mode,repeat;Trial result;};std::vector<Measured> measured;measured.reserve(9);
   for(int repeat=0;repeat<3;++repeat)for(const auto& m:request["mode_order"])measured.push_back({m.asInt(),repeat,trial(in,m.asInt())});
   // Oracle recomputation AFTER the timed series, never an uncounted pre-score cache.
   for(auto& item:measured){item.result.oracleMatch=verify(in,item.result);if(!item.result.oracleMatch)throw std::runtime_error("completed gain differs from full recomputation");output["trials"].append(trialJson(item.result,item.mode,item.repeat));}
  }catch(const ContextError& e){output["status"]="REPLAY_CONTEXT_INVALID";output["reason"]=e.what();}
 }
 Json::StreamWriterBuilder writer;writer["indentation"]="";std::cout<<Json::writeString(writer,output)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
