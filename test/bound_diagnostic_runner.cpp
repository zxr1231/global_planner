#include "snapshotFixtureIO.h"
#include "boundDiagnostic.h"
#include "fixtureAllocation.h"
#include <chrono>
using namespace boundDiagnostic;
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
Vec vec(const Json::Value& a){return {{number(a[0]),number(a[1]),number(a[2])}};}
Json::Value boxJson(const Box& b){Json::Value j;j["lo"]=Json::Value(Json::arrayValue);j["hi"]=Json::Value(Json::arrayValue);for(int k=0;k<3;++k){j["lo"].append(b.lo[k]);j["hi"].append(b.hi[k]);}return j;}
int main(int argc,char** argv){try{
 if(argc!=3)throw std::invalid_argument("usage: bound_diagnostic_runner SNAPSHOT REQUEST");
 auto io=Clock::now();auto req=loadJson(argv[2]);Json::Value out;
 if(!req["accepted"].asBool()){out["status"]="SKIP_NEGATIVE_CONTROL";out["camera_queries"]=0;out["tables_built"]=0;}
 else{
  std::string dir=argv[1];auto manifest=loadJson(dir+"/manifest.json");if(!manifest["complete"].asBool()||manifest["schema"].asString()!="cerlab-r1-snapshot-v1")throw std::runtime_error("manifest invalid");
  auto mapfile=dir+"/"+manifest["map_file"].asString(),plannerfile=dir+"/"+manifest["planner_file"].asString();if(hex64(fnv1a64(mapfile))!=manifest["map_fnv1a64"].asString()||hex64(fnv1a64(plannerfile))!=manifest["planner_fnv1a64"].asString())throw std::runtime_error("source hash mismatch");
  auto map=loadMap(mapfile);validateMap(map);auto planner=loadJson(plannerfile);if(map.version!=req["map_version"].asUInt64()||map.version!=manifest["map_version"].asUInt64()||map.version!=planner["map_version"].asUInt64())throw std::runtime_error("map version mismatch");
  std::vector<uint32_t> reference;for(auto n:req["reference_addresses"])reference.push_back(n.asUInt());if(!std::is_sorted(reference.begin(),reference.end())||std::adjacent_find(reference.begin(),reference.end())!=reference.end())throw std::runtime_error("reference addresses not unique sorted");
  auto low=vec(req["camera"]["planning_min"]),high=vec(req["camera"]["planning_max"]);auto roi=region(map,low,high);
  for(auto id:reference)if(!inside(roi,index(map,id))||map.occupancy[id]>=map.pMinLog)throw std::runtime_error("reference not Unknown");
  auto c=req["camera"];double offset=std::hypot(std::hypot(number(c["body_to_camera"][3]),number(c["body_to_camera"][7])),number(c["body_to_camera"][11]));double radius=number(c["raycast_max_length"])+offset+std::sqrt(3.)*map.resolution;
  std::vector<Route> routes;for(auto x:req["routes"]){Route r;r.id=x["record_id"].asUInt64();r.anchor=x["anchor_id"].asUInt64();r.e=x["reference_segment"].asUInt64();r.time=number(x["motion"]["time"]);r.length=number(x["motion"]["length"]);for(auto s:x["segments"])r.boxes.push_back(envelope(map,vec(s[0]),vec(s[1]),radius));routes.push_back(r);}
  size_t routeStorage=routes.capacity()*sizeof(Route);for(const auto& r:routes)routeStorage+=r.boxes.capacity()*sizeof(Box);
  out["preloaded_reference_storage_bytes"]=Json::UInt64(reference.capacity()*sizeof(uint32_t));out["preloaded_route_storage_bytes"]=Json::UInt64(routeStorage);
  out["input_io_and_geometry_ms"]=ms(io);out["radius_m"]=radius;out["roi"]=boxJson(roi);out["map_version"]=Json::UInt64(map.version);out["input_map_payload_bytes"]=Json::UInt64(map.occupancy.capacity()*sizeof(double)+(map.inflated.capacity()+7)/8);out["camera_model_constructed"]=false;out["camera_queries"]=0;out["new_routes_generated"]=0;out["rows"]=Json::Value(Json::arrayValue);
  for(auto r:routes){Json::Value j;j["record_id"]=Json::UInt64(r.id);j["boxes"]=Json::Value(Json::arrayValue);for(auto b:r.boxes)j["boxes"].append(boxJson(b));out["rows"].append(j);}
  auto motion=routes;std::sort(motion.begin(),motion.end(),[](const Route& a,const Route& b){return stableKey(a)<stableKey(b);});out["motion_order"]=Json::Value(Json::arrayValue);for(auto r:motion)out["motion_order"].append(Json::UInt64(r.id));
  for(bool conditioned:{false,true}){
   fixtureAllocation::Ledger ledger;std::vector<Scored> scores;double build=0,query=0,sort=0,total=0;int64_t nTotal=0;size_t tableBytes=0;
   if(!routes.empty()){
    auto t0=Clock::now();{
     fixtureAllocation::Scope scope(ledger);auto t=Clock::now();Prefix table(map,low,high,reference,conditioned);build=ms(t);tableBytes=table.storageBytes();nTotal=table.total();
     t=Clock::now();for(auto& r:routes)scores.push_back(score(table,r,reference.size()));query=ms(t);
     t=Clock::now();std::sort(scores.begin(),scores.end(),[&](const Scored& a,const Scored& b){if(a.upper!=b.upper)return a.upper>b.upper;auto ia=std::find_if(routes.begin(),routes.end(),[&](const Route& r){return r.id==a.id;}),ib=std::find_if(routes.begin(),routes.end(),[&](const Route& r){return r.id==b.id;});return stableKey(*ia)<stableKey(*ib);});sort=ms(t);
    }total=ms(t0);
   }
   Json::Value j;j["index_build_ms"]=build;j["union_queries_ms"]=query;j["sort_ms"]=sort;j["total_preparation_ms"]=total;j["tracked_scratch_peak_bytes"]=Json::UInt64(ledger.peak);j["table_storage_bytes"]=Json::UInt64(tableBytes);j["mask_total"]=Json::Int64(nTotal);j["allocation_limit_rejected"]=ledger.rejected;j["order"]=Json::Value(Json::arrayValue);j["scores"]=Json::Value(Json::arrayValue);
   for(auto s:scores){j["order"].append(Json::UInt64(s.id));Json::Value row;row["record_id"]=Json::UInt64(s.id);row["count"]=Json::Int64(s.count);row["upper_utility"]=s.upper;j["scores"].append(row);}out[conditioned?"Q":"N"]=j;
  }out["status"]="COMPLETE";
 }
 Json::StreamWriterBuilder w;w["indentation"]="";std::cout<<Json::writeString(w,out)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
