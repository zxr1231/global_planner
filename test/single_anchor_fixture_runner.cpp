#include "snapshotFixtureIO.h"
#include "singleAnchorFixture.h"
using namespace singleAnchorFixture;
Json::Value sequence(const Sequence& path){Json::Value v(Json::arrayValue);for(auto id:path)v.append(Json::UInt64(id));return v;}
Json::Value motionJson(const Motion& m){Json::Value v;v["length"]=m.length;v["time"]=m.time;v["yaw"]=m.yaw;return v;}
Json::Value samplesJson(const std::vector<PathGainWaypoint>& poses){Json::Value v(Json::arrayValue);for(const auto& pose:poses){Json::Value a;a["position"]=Json::Value(Json::arrayValue);for(int k=0;k<3;++k)a["position"].append(pose.position[k]);a["yaw"]=pose.yaw;v.append(a);}return v;}
int main(int argc,char** argv){try{
 if(argc!=4)throw std::invalid_argument("usage: single_anchor_fixture_runner pool|gain SNAPSHOT REQUEST");
 const std::string mode=argv[1],dir=argv[2];auto request=loadJson(argv[3]);auto manifest=loadJson(dir+"/manifest.json");
 if(manifest["schema"].asString()!="cerlab-r1-snapshot-v1"||!manifest["complete"].asBool())throw std::runtime_error("invalid manifest");
 auto mapfile=dir+"/"+manifest["map_file"].asString(),plannerfile=dir+"/"+manifest["planner_file"].asString();
 if(hex64(fnv1a64(mapfile))!=manifest["map_fnv1a64"].asString()||hex64(fnv1a64(plannerfile))!=manifest["planner_fnv1a64"].asString())throw std::runtime_error("snapshot hash mismatch");
 auto planner=loadJson(plannerfile);Input in;in.map=loadMap(mapfile);in.expectedVersion=request["map_version"].asUInt64();
 if(in.expectedVersion!=in.map.version||in.map.version!=manifest["map_version"].asUInt64()||in.map.version!=planner["map_version"].asUInt64())throw ContextError("MAP_VERSION_MISMATCH");
 in.accepted=request["accepted"].asBool();Json::Value out;
 if(!in.accepted){out["status"]="SKIP_B_NEGATIVE_CONTROL";out["camera_calls"]=0;out["records"]=Json::Value(Json::arrayValue);}
 else {
 const auto c=request["camera"];in.camera.fx=number(c["fx"]);in.camera.fy=number(c["fy"]);in.camera.cx=number(c["cx"]);in.camera.cy=number(c["cy"]);in.camera.cols=c["image_cols"].asInt();in.camera.rows=c["image_rows"].asInt();in.camera.margin=c["filter_margin"].asInt();in.camera.pixelSkip=c["evaluation_pixel_skip"].asInt();in.camera.range=number(c["raycast_max_length"]);
 for(int r=0;r<4;++r)for(int k=0;k<4;++k)in.camera.bodyToCamera(r,k)=number(c["body_to_camera"][4*r+k]);
 in.legacy.horizontalFov=number(planner["sensor_model"]["horizontal_fov"]);in.legacy.verticalFov=number(planner["sensor_model"]["vertical_fov"]);in.legacy.dmax=number(planner["sensor_model"]["dmax"]);
 for(int k=0;k<3;++k){in.camera.planningMin[k]=in.legacy.planningMin[k]=number(c["planning_min"][k]);in.camera.planningMax[k]=in.legacy.planningMax[k]=number(c["planning_max"][k]);}
 in.motion.speed=number(planner["motion_model"]["velocity"]);in.motion.angularSpeed=number(planner["motion_model"]["angular_velocity"]);in.motion.yawPenalty=number(planner["motion_model"]["yaw_penalty_weight"]);
 if(mode=="pool"){
  in.initialYaw=number(request["start_yaw"]);in.terminalYaw=number(request["terminal_yaw"]);in.goal=request["goal"].asUInt64();
  for(const auto& v:request["nodes"])in.positions.push_back({number(v[0]),number(v[1]),number(v[2])});
  for(const auto& v:request["edges"])in.edges.push_back({v[0].asUInt64(),v[1].asUInt64()});
  for(const auto& v:request["raw"])in.raw.push_back(v.asUInt64());
  for(const auto& v:request["reference"])in.reference.push_back(v.asUInt64());
  auto begin=Clock::now();auto supply=generateAnchors(in);out["generation_component_ms"]=elapsed(begin);
  out["status"]="CONTEXT_VALID";out["camera_calls"]=0;out["map_version"]=Json::UInt64(in.map.version);out["reference_motion"]=motionJson(supply.referenceMotion);out["reference_samples"]=samplesJson(sample(in,in.reference));out["possible_pairs"]=Json::UInt64(supply.possiblePairs);out["truncated"]=supply.truncated;
  out["ordered"]=Json::Value(Json::arrayValue);for(auto i:supply.ordered)out["ordered"].append(Json::UInt64(i));out["pool"]=Json::Value(Json::arrayValue);for(auto i:supply.pool)out["pool"].append(Json::UInt64(i));out["records"]=Json::Value(Json::arrayValue);
  for(size_t i=0;i<supply.records.size();++i){const auto& r=supply.records[i];Json::Value v;v["record_id"]=Json::UInt64(i);v["edge"]=Json::UInt64(r.edge);v["anchor"]=Json::UInt64(r.anchor);v["path"]=sequence(r.path);v["motion_measured"]=r.motionMeasured;if(r.motionMeasured)v["motion"]=motionJson(r.motion);v["reasons"]=Json::Value(Json::arrayValue);for(const auto& reason:r.reasons)v["reasons"].append(reason);
   if(std::find(supply.pool.begin(),supply.pool.end(),i)!=supply.pool.end()){v["samples"]=samplesJson(sample(in,r.path));const auto a=position(in,r.anchor),b=position(in,in.reference[r.edge+1]);v["anchor_yaw"]=std::atan2(b.y()-a.y(),b.x()-a.x());}
   out["records"].append(v);
  }
 }else if(mode=="gain"){
  CameraGainEvaluator evaluator(in.map,in.camera);std::vector<PathGainWaypoint> samples;
  for(const auto& v:request["samples"]){PathGainWaypoint p;for(int k=0;k<3;++k)p.position[k]=number(v["position"][k]);p.yaw=number(v["yaw"]);samples.push_back(p);}
  const auto result=evaluator.evaluateSamples(samples,number(request["estimated_time"]),in.expectedVersion);
  if(!result.gain.valid)throw std::runtime_error(result.failureReason);
  out["valid"]=true;out["map_version"]=Json::UInt64(in.map.version);out["raw_gain"]=Json::UInt64(result.gain.rawGain);out["unique_gain"]=Json::UInt64(result.gain.uniqueGain);out["utility"]=result.gain.uniqueUtility;out["evaluation_component_ms"]=result.evaluationMs;out["camera_calls"]=1;
  out["unique_addresses"]=Json::Value(Json::arrayValue);for(auto a:result.gain.uniqueAddresses)out["unique_addresses"].append(Json::UInt(a));out["marginal_gains"]=Json::Value(Json::arrayValue);for(auto a:result.gain.marginalGains)out["marginal_gains"].append(Json::UInt64(a));
 }else throw std::invalid_argument("invalid mode");
 }
 Json::StreamWriterBuilder w;w["indentation"]="";std::cout<<Json::writeString(w,out)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
