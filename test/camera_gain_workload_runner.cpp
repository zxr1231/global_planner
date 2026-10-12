#include "snapshotFixtureIO.h"
#include <global_planner/cameraGainEvaluator.h>
#include <memory>
#include <chrono>
#include <algorithm>
#include <sys/resource.h>
using Clock=std::chrono::steady_clock;
using Samples=std::vector<globalPlanner::PathGainWaypoint>;
double ms(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
struct Route {int id;double time;Samples full,translation;};
Samples samples(const Json::Value& array){Samples out;for(const auto& x:array){globalPlanner::PathGainWaypoint p;for(int a=0;a<3;++a)p.position[a]=number(x["position"][a]);p.yaw=number(x["yaw"]);out.push_back(p);}return out;}
Json::Value gainRow(int id,const globalPlanner::CameraGainResult& r){Json::Value j;j["id"]=id;j["valid"]=r.gain.valid;j["raw_gain"]=Json::UInt64(r.gain.rawGain);j["unique_gain"]=Json::UInt64(r.gain.uniqueGain);j["sample_count"]=r.gain.sampleCount;j["evaluation_ms"]=r.evaluationMs;j["failure_reason"]=r.failureReason;j["marginals"]=Json::Value(Json::arrayValue);for(auto n:r.gain.marginalGains)j["marginals"].append(Json::UInt64(n));return j;}
int main(int argc,char** argv){
 try {
  if(argc!=3)throw std::invalid_argument("usage: camera_gain_workload_runner SNAPSHOT_DIR REQUEST_JSON");
  const std::string dir=argv[1];const auto manifest=loadJson(dir+"/manifest.json");
  if(!manifest["complete"].asBool() || manifest["schema"].asString()!="cerlab-r1-snapshot-v1")throw std::runtime_error("invalid manifest");
  const std::string mapFile=dir+"/"+manifest["map_file"].asString(),plannerFile=dir+"/"+manifest["planner_file"].asString();
  if(hex64(fnv1a64(mapFile))!=manifest["map_fnv1a64"].asString() || hex64(fnv1a64(plannerFile))!=manifest["planner_fnv1a64"].asString())throw std::runtime_error("snapshot integrity failure");
  const auto frozen=loadMap(mapFile); // immutable source; stage copies explicitly timed below
  const auto planner=loadJson(plannerFile);
  if(frozen.version!=manifest["map_version"].asUInt64() || frozen.version!=planner["map_version"].asUInt64())throw std::runtime_error("snapshot version mismatch");
  const auto input=loadJson(argv[2]),j=input["config"];
  globalPlanner::CameraGainConfig config;config.fx=number(j["fx"]);config.fy=number(j["fy"]);config.cx=number(j["cx"]);config.cy=number(j["cy"]);config.cols=j["image_cols"].asInt();config.rows=j["image_rows"].asInt();config.margin=j["filter_margin"].asInt();config.pixelSkip=j["evaluation_pixel_skip"].asInt();config.range=number(j["raycast_max_length"]);
  for(int r=0;r<4;++r)for(int c=0;c<4;++c)config.bodyToCamera(r,c)=number(j["body_to_camera"][r*4+c]);
  for(int a=0;a<3;++a){config.planningMin[a]=number(j["planning_min"][a]);config.planningMax[a]=number(j["planning_max"][a]);}
  std::vector<Route> routes;for(const auto& r:input["routes"])routes.push_back({r["id"].asInt(),number(r["estimated_time"]),samples(r["samples"]),samples(r["translation_only_samples"])});
  if(routes.empty())throw std::runtime_error("no references");
  for(size_t i=1;i<routes.size();++i)if(routes[i-1].id>=routes[i].id)throw std::runtime_error("reference order must be stable and unique");
  const auto warm=Clock::now();globalPlanner::CameraGainEvaluator warmed(frozen,config);
  for(const auto& r:routes)if(!warmed.evaluateSamples(r.full,r.time,frozen.version).gain.valid)throw std::runtime_error("warmup invalid");
  Json::Value output;output["warmup_ms"]=ms(warm);output["map_version"]=Json::UInt64(frozen.version);output["actual_candidates"]=Json::UInt64(routes.size());output["repeats"]=Json::Value(Json::arrayValue);
  for(int repeat=0;repeat<input["repeats"].asInt();++repeat){
   Json::Value row;auto begin=Clock::now();auto map=frozen;row["copy_ms"]=ms(begin);auto construct=Clock::now();globalPlanner::CameraGainEvaluator evaluator(map,config);row["construction_ms"]=ms(construct);row["preparation_ms"]=ms(begin);
   Json::Value results(Json::arrayValue);auto scoreBegin=Clock::now();std::vector<double> prefixes;
   for(const auto& r:routes){auto g=evaluator.evaluateSamples(r.full,r.time,map.version);if(!g.gain.valid)throw std::runtime_error(g.failureReason);auto scored=gainRow(r.id,g);prefixes.push_back(ms(begin));scored["cumulative_preparation_scoring_ms"]=prefixes.back();results.append(scored);}
   row["scoring_ms"]=ms(scoreBegin);row["preparation_scoring_ms"]=ms(begin);row["routes"]=results;
   row["count_diagnostics"]=Json::Value(Json::arrayValue);
   for(const auto& n:input["count_diagnostics"]){Json::Value x;x["requested"]=n;x["available"]=n.asUInt64()<=routes.size();if(x["available"].asBool()){x["cumulative_ms"]=prefixes[n.asUInt64()-1];x["complete_goal_set"]=n.asUInt64()==routes.size();}row["count_diagnostics"].append(x);}
   auto secondary=Clock::now();Json::Value light(Json::arrayValue);
   for(const auto& r:routes){auto g=evaluator.evaluateSamples(r.translation,r.time,map.version);if(!g.gain.valid)throw std::runtime_error(g.failureReason);light.append(gainRow(r.id,g));}
   row["translation_only_scoring_ms"]=ms(secondary);row["translation_only_routes"]=light;output["repeats"].append(row);
  }
  // Guard is an offline simulation of the candidate stage, not an online controller.
  auto guarded=Clock::now();auto map=frozen;globalPlanner::CameraGainEvaluator evaluator(map,config);Json::Value guard;guard["preparation_ms"]=ms(guarded);guard["routes"]=Json::Value(Json::arrayValue);size_t completed=0;
  for(const auto& r:routes){const double remaining=number(input["budget_ms"])-ms(guarded);if(remaining<=0){guard["stop_reason"]="shared_budget_before_route";break;}
   globalPlanner::CameraGainBudget b;b.maxMilliseconds=remaining;auto g=evaluator.evaluateSamples(r.full,r.time,map.version,b);guard["routes"].append(gainRow(r.id,g));
   if(!g.gain.valid){if(g.gain.rawGain || g.gain.uniqueGain || !g.gain.uniqueAddresses.empty() || !g.gain.marginalGains.empty())throw std::runtime_error("invalid partial gain escaped");guard["stop_reason"]=g.failureReason;break;}++completed;
  }
  guard["completed_references"]=Json::UInt64(completed);guard["required_references"]=Json::UInt64(routes.size());guard["complete_goal_set"]=completed==routes.size();guard["fallback"]=completed!=routes.size();guard["elapsed_ms"]=ms(guarded);guard["overshoot_ms"]=std::max(0.,guard["elapsed_ms"].asDouble()-number(input["budget_ms"]));output["budget_guard"]=guard;
  struct rusage usage{};getrusage(RUSAGE_SELF,&usage);output["peak_rss_mib"]=double(usage.ru_maxrss)/1024;
  Json::StreamWriterBuilder writer;writer["indentation"]="";std::cout<<Json::writeString(writer,output)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
