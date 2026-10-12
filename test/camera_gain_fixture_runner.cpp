#include "snapshotFixtureIO.h"
#include <global_planner/cameraGainEvaluator.h>
#include <chrono>
int main(int argc,char** argv) {
 try {
  if(argc!=3)throw std::invalid_argument("usage: camera_gain_fixture_runner SNAPSHOT_DIR INPUT_JSON");
  const std::string directory=argv[1];const auto manifest=loadJson(directory+"/manifest.json");
  if(manifest["schema"].asString()!="cerlab-r1-snapshot-v1" || !manifest["complete"].asBool())throw std::runtime_error("invalid manifest");
  const std::string file=directory+"/"+manifest["map_file"].asString(),plannerFile=directory+"/"+manifest["planner_file"].asString();
  if(hex64(fnv1a64(file))!=manifest["map_fnv1a64"].asString() || hex64(fnv1a64(plannerFile))!=manifest["planner_fnv1a64"].asString())throw std::runtime_error("snapshot hash mismatch");
  const auto map=loadMap(file);const auto planner=loadJson(plannerFile);
  if(map.version!=manifest["map_version"].asUInt64() || map.version!=planner["map_version"].asUInt64())throw std::runtime_error("snapshot version mismatch");
  const auto input=loadJson(argv[2]),j=input["config"];
  globalPlanner::CameraGainConfig c;c.fx=number(j["fx"]);c.fy=number(j["fy"]);c.cx=number(j["cx"]);c.cy=number(j["cy"]);
  c.cols=j["image_cols"].asInt();c.rows=j["image_rows"].asInt();c.margin=j["filter_margin"].asInt();c.pixelSkip=j["evaluation_pixel_skip"].asInt();c.range=number(j["raycast_max_length"]);
  for(int row=0;row<4;++row)for(int col=0;col<4;++col)c.bodyToCamera(row,col)=number(j["body_to_camera"][4*row+col]);
  for(int a=0;a<3;++a){c.planningMin[a]=number(j["planning_min"][a]);c.planningMax[a]=number(j["planning_max"][a]);}
  const auto begun=std::chrono::steady_clock::now();globalPlanner::CameraGainEvaluator evaluator(map,c);
  const double construction=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begun).count();
  std::vector<globalPlanner::PathGainWaypoint> samples;
  for(const auto& j:input["samples"]){globalPlanner::PathGainWaypoint p;for(int a=0;a<3;++a)p.position[a]=number(j["position"][a]);p.yaw=number(j["yaw"]);samples.push_back(p);}
  const double time=number(input["estimated_time"]);
  Json::Value output;output["model"]=globalPlanner::CameraGainConfig::model;output["schema"]=1;output["construction_ms"]=construction;output["ray_count"]=Json::UInt64(evaluator.rayCount());
  auto evaluate=[&]{return evaluator.evaluateSamples(samples,time,map.version);};
  auto warmup=evaluate();if(!warmup.gain.valid)throw std::runtime_error(warmup.failureReason);
  globalPlanner::CameraGainResult result;
  for(int i=0;i<3;++i){result=evaluate();if(!result.gain.valid)throw std::runtime_error(result.failureReason);output["timing_ms"].append(result.evaluationMs);}
  output["valid"]=result.gain.valid;output["map_version"]=Json::UInt64(result.gain.mapVersion);output["raw_gain"]=Json::UInt64(result.gain.rawGain);output["unique_gain"]=Json::UInt64(result.gain.uniqueGain);output["voxel_visits"]=Json::UInt64(result.voxelVisits);
  output["unique_addresses"]=Json::Value(Json::arrayValue);output["marginal_gains"]=Json::Value(Json::arrayValue);
  for(auto id:result.gain.uniqueAddresses)output["unique_addresses"].append(Json::UInt(id));
  for(auto n:result.gain.marginalGains)output["marginal_gains"].append(Json::UInt64(n));
  // Serialized fixed-pose sets are a correctness oracle, outside reported evaluation times.
  output["pose_addresses"]=Json::Value(Json::arrayValue);
  for(const auto& pose:samples) {
   const auto visible=evaluator.visibleUnknown(pose.position,pose.yaw);std::vector<uint32_t> ordered(visible.begin(),visible.end());std::sort(ordered.begin(),ordered.end());
   Json::Value ids(Json::arrayValue);for(auto id:ordered)ids.append(Json::UInt(id));output["pose_addresses"].append(ids);
  }
  Json::StreamWriterBuilder writer;writer["indentation"]="";std::cout<<Json::writeString(writer,output)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
