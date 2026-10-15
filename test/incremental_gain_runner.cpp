#include "snapshotFixtureIO.h"
#include "fixtureAllocation.h"
#include <global_planner/incrementalGainEvaluator.h>
#include <sys/resource.h>
#include <fstream>
using namespace globalPlanner;
using Clock=std::chrono::steady_clock;
namespace {
double ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
CameraGainConfig config(const Json::Value& j){CameraGainConfig c;c.fx=number(j["fx"]);c.fy=number(j["fy"]);c.cx=number(j["cx"]);c.cy=number(j["cy"]);c.range=number(j["raycast_max_length"]);c.cols=j["image_cols"].asInt();c.rows=j["image_rows"].asInt();c.margin=j["filter_margin"].asInt();c.pixelSkip=j["evaluation_pixel_skip"].asInt();for(int r=0;r<4;++r)for(int k=0;k<4;++k)c.bodyToCamera(r,k)=number(j["body_to_camera"][4*r+k]);for(int a=0;a<3;++a){c.planningMin[a]=number(j["planning_min"][a]);c.planningMax[a]=number(j["planning_max"][a]);}return c;}
Json::Value gain(const PathGainEvaluation& g){Json::Value j;j["raw"]=Json::UInt64(g.rawGain);j["unique"]=Json::UInt64(g.uniqueGain);j["utility"]=g.uniqueUtility;j["sample_count"]=g.sampleCount;j["map_version"]=Json::UInt64(g.mapVersion);j["marginal"]=Json::Value(Json::arrayValue);for(auto n:g.marginalGains)j["marginal"].append(Json::UInt64(n));j["addresses"]=Json::Value(Json::arrayValue);for(auto n:g.uniqueAddresses)j["addresses"].append(n);return j;}
}
int main(int argc,char** argv){try{
 if(argc!=4)throw std::invalid_argument("usage: incremental_gain_runner INPUT_JSON MODE OUTPUT_JSONL");
 int mode=std::stoi(argv[2]);if(mode<0||mode>3)throw std::invalid_argument("mode");
 auto input=loadJson(argv[1]);std::ofstream output(argv[3]);if(!output)throw std::runtime_error("output");
 fixtureAllocation::Ledger ledger;ledger.limit=2ull*1024*1024*1024; // Tracks full tool working set; retained cache is separately capped at 256 MiB.
 fixtureAllocation::Scope scope(ledger);IncrementalGainEvaluator evaluator(static_cast<IncrementalGainMode>(mode));
 for(const auto& event:input["events"]){
  auto io=Clock::now();auto request=loadJson(event["request"].asString());auto original=loadMap(event["map"].asString());auto c=config(request["config"]);std::vector<IncrementalGainRoute> routes;
  for(const auto& j:request["routes"]){IncrementalGainRoute r;r.id=j["id"].asUInt64();r.estimatedTime=number(j["estimated_time"]);for(const auto& q:j["samples"]){PathGainWaypoint p;for(int a=0;a<3;++a)p.position[a]=number(q["position"][a]);p.yaw=number(q["yaw"]);r.samples.push_back(p);}routes.push_back(std::move(r));}
  double ioMs=ms(io);auto begin=Clock::now();auto m=original;auto result=evaluator.evaluate(m,c,routes);m=mapManager::OccupancyMapSnapshot();double total=ms(begin);
  const auto& s=result.stats;Json::Value j;j["sequence"]=event["sequence"];j["mode"]=mode;j["input_io_ms_excluded"]=ioMs;j["total_ms"]=total;j["evaluator_ms"]=s.totalMs;j["preparation_ms"]=s.preparationMs;j["difference_ms"]=s.differenceMs;j["update_ms"]=s.updateMs;j["path_ms"]=s.pathMs;j["changed_voxels"]=Json::UInt64(s.changedVoxels);j["blocking_changes"]=Json::UInt64(s.blockingChanges);j["requested_samples"]=Json::UInt64(s.requestedSamples);j["distinct_poses"]=Json::UInt64(s.distinctPoses);j["recomputed_poses"]=Json::UInt64(s.recomputedPoses);j["reused_poses"]=Json::UInt64(s.reusedPoses);j["membership_updates"]=Json::UInt64(s.membershipUpdates);j["propagated_memberships"]=Json::UInt64(s.propagatedMemberships);j["evictions"]=Json::UInt64(s.evictions);j["charged_cache_bytes"]=Json::UInt64(s.cacheBytes);j["reset"]=s.reset;j["winner"]=result.winner;
  struct rusage usage{};getrusage(RUSAGE_SELF,&usage);j["process_peak_rss_kib"]=Json::UInt64(usage.ru_maxrss);j["tracked_working_peak_bytes"]=Json::UInt64(ledger.peak);j["allocation_limit_rejected"]=ledger.rejected;j["routes"]=Json::Value(Json::arrayValue);for(const auto& g:result.routes)j["routes"].append(gain(g));
  Json::StreamWriterBuilder w;w["indentation"]="";output<<Json::writeString(w,j)<<"\n";output.flush();if(!output)throw std::runtime_error("write");std::cerr<<"sequence "<<event["sequence"].asInt()<<" mode "<<mode<<" ms "<<total<<"\n";
 }
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
