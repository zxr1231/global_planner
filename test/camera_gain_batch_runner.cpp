#include "snapshotFixtureIO.h"
#include <global_planner/cameraGainEvaluator.h>
#include <memory>
#include <chrono>
#include <sys/resource.h>
struct Context {
 mapManager::OccupancyMapSnapshot map;
 std::unique_ptr<globalPlanner::CameraGainEvaluator> evaluator;
 std::vector<globalPlanner::PathGainWaypoint> samples;
 double time=0;
};
int main(int argc,char** argv) {
 try {
  if(argc!=2 && argc!=3)throw std::invalid_argument("usage: camera_gain_batch_runner REQUEST_JSON [--profile]");
  globalPlanner::CameraGainBudget budget;budget.profile=argc==3 && std::string(argv[2])=="--profile";
  const auto requests=loadJson(argv[1]);if(!requests.isArray() || requests.size()!=12)throw std::invalid_argument("registered workload needs12 prefixes");
  std::vector<std::unique_ptr<Context>> contexts;
  const auto preparation=std::chrono::steady_clock::now();
  for(const auto& item:requests) {
   const auto input=loadJson(item["input"].asString());const std::string dir=item["snapshot"].asString();const auto manifest=loadJson(dir+"/manifest.json");
   if(!manifest["complete"].asBool() || manifest["schema"].asString()!="cerlab-r1-snapshot-v1")throw std::runtime_error("invalid map manifest");
   const std::string file=dir+"/"+manifest["map_file"].asString();
   if(hex64(fnv1a64(file))!=manifest["map_fnv1a64"].asString())throw std::runtime_error("map hash mismatch");
   auto ctx=std::make_unique<Context>();ctx->map=loadMap(file);
   if(ctx->map.version!=manifest["map_version"].asUInt64())throw std::runtime_error("map version mismatch");
   const auto j=input["config"];globalPlanner::CameraGainConfig c;
   c.fx=number(j["fx"]);c.fy=number(j["fy"]);c.cx=number(j["cx"]);c.cy=number(j["cy"]);c.range=number(j["raycast_max_length"]);c.cols=j["image_cols"].asInt();c.rows=j["image_rows"].asInt();c.margin=j["filter_margin"].asInt();c.pixelSkip=j["evaluation_pixel_skip"].asInt();
   for(int r=0;r<4;++r)for(int k=0;k<4;++k)c.bodyToCamera(r,k)=number(j["body_to_camera"][4*r+k]);
   for(int a=0;a<3;++a){c.planningMin[a]=number(j["planning_min"][a]);c.planningMax[a]=number(j["planning_max"][a]);}
   ctx->evaluator=std::make_unique<globalPlanner::CameraGainEvaluator>(ctx->map,c);
   for(const auto& sample:input["samples"]){globalPlanner::PathGainWaypoint p;for(int a=0;a<3;++a)p.position[a]=number(sample["position"][a]);p.yaw=number(sample["yaw"]);ctx->samples.push_back(p);}
   ctx->time=number(input["estimated_time"]);contexts.push_back(std::move(ctx));
  }
  Json::Value output;output["preparation_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-preparation).count();
  for(const auto& ctx:contexts)if(!ctx->evaluator->evaluateSamples(ctx->samples,ctx->time,ctx->map.version).gain.valid)throw std::runtime_error("batch warmup invalid");
  const auto started=std::chrono::steady_clock::now();
  for(int i=0;i<30;++i) {
   const auto& c=*contexts[i%12];auto result=c.evaluator->evaluateSamples(c.samples,c.time,c.map.version,budget);
   if(!result.gain.valid)throw std::runtime_error(result.failureReason);
   Json::Value row;row["index"]=i;row["prefix_index"]=i%12;row["evaluation_ms"]=result.evaluationMs;row["endpoint_ms"]=result.endpointMs;row["traversal_ms"]=result.traversalMs;row["representatives"]=Json::UInt64(result.representativeCount);row["visits"]=Json::UInt64(result.voxelVisits);row["raw_gain"]=Json::UInt64(result.gain.rawGain);row["unique_gain"]=Json::UInt64(result.gain.uniqueGain);output["candidates"].append(row);
  }
  output["batch_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  output["passed_500ms"]=output["batch_ms"].asDouble()<=500;
  struct rusage usage{};getrusage(RUSAGE_SELF,&usage);output["peak_rss_mib_12_snapshots"]=double(usage.ru_maxrss)/1024;
  Json::StreamWriterBuilder writer;writer["indentation"]="";std::cout<<Json::writeString(writer,output)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
