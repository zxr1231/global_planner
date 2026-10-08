#include <global_planner/routeSnapshot.h>
#include <global_planner/routeCandidates.h>
#include <json/json.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <chrono>
namespace{

template<typename T>
T readValue(std::istream& stream){
	T value;
	stream.read(reinterpret_cast<char*>(&value),sizeof(T));
	if (!stream) throw std::runtime_error("truncated map binary");
	return value;
}

uint64_t fnv1a64(const std::string& path){
	std::ifstream stream(path,std::ios::binary);
	if (!stream) throw std::runtime_error("cannot open file for hash");
	uint64_t value = 14695981039346656037ull;
	char buffer[65536];
	while (stream){
		stream.read(buffer,sizeof(buffer));
		for (std::streamsize index=0; index<stream.gcount(); ++index){
			value ^= static_cast<unsigned char>(buffer[index]);
			value *= 1099511628211ull;
		}
	}
	return value;
}

std::string hex64(uint64_t value){
	std::ostringstream stream;
	stream << std::hex << std::setfill('0') << std::setw(16) << value;
	return stream.str();
}

Json::Value loadJson(const std::string& path){
	std::ifstream stream(path);
	if (!stream) throw std::runtime_error("cannot open json");
	Json::CharReaderBuilder builder;
	Json::Value value;
	std::string errors;
	if (!Json::parseFromStream(builder,stream,&value,&errors)){
		throw std::runtime_error("invalid json: "+errors);
	}
	return value;
}

mapManager::OccupancyMapSnapshot loadMap(const std::string& path){
	std::ifstream stream(path,std::ios::binary);
	if (!stream) throw std::runtime_error("cannot open map binary");
	char magic[8]; stream.read(magic,sizeof(magic));
	if (!stream || std::string(magic,7)!="CR1MAP1") throw std::runtime_error("invalid map magic");
	if (readValue<uint32_t>(stream)!=1) throw std::runtime_error("unsupported map schema");
	mapManager::OccupancyMapSnapshot snapshot;
	snapshot.version = readValue<uint64_t>(stream);
	snapshot.resolution = readValue<double>(stream);
	for (int axis=0; axis<3; ++axis) snapshot.mapMin(axis)=readValue<double>(stream);
	for (int axis=0; axis<3; ++axis) snapshot.mapMax(axis)=readValue<double>(stream);
	for (int axis=0; axis<3; ++axis) snapshot.dimensions(axis)=readValue<int32_t>(stream);
	snapshot.pMinLog=readValue<double>(stream);
	snapshot.pOccLog=readValue<double>(stream);
	const uint64_t count=readValue<uint64_t>(stream);
	if (count!=static_cast<uint64_t>(snapshot.dimensions(0))*snapshot.dimensions(1)*
		snapshot.dimensions(2)) throw std::runtime_error("map voxel count mismatch");
	snapshot.occupancy.resize(count);
	snapshot.inflated.resize(count);
	for (uint64_t index=0; index<count; ++index){
		const int8_t state=readValue<int8_t>(stream);
		const uint8_t inflated=readValue<uint8_t>(stream);
		if (state < -1 || state > 1 || inflated > 1) throw std::runtime_error("invalid map payload");
		snapshot.occupancy[index] = state == -1 ? snapshot.pMinLog-1.0 :
			(state == 1 ? snapshot.pOccLog : snapshot.pMinLog);
		snapshot.inflated[index] = inflated != 0;
	}
	if (stream.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("trailing map data");
	return snapshot;
}

double number(const Json::Value& value){
	if (!value.isNumeric()) throw std::runtime_error("expected numeric json value");
	return value.asDouble();
}

}


namespace {
using namespace globalPlanner;
using namespace globalPlanner::routeSearch;
using namespace globalPlanner::routeCandidates;
Json::Value sequence(const Sequence& path){Json::Value v(Json::arrayValue);for(Id n:path)v.append(Json::UInt64(n));return v;}
Json::Value motionJson(const Motion& m){Json::Value v;v["length"]=m.length;v["time"]=m.time;v["yaw"]=m.yaw;return v;}
GoalPool diagnosticPool(const GoalPool& input,const Graph& graph,const Limits& limits,bool postOnly) {
 GoalPool out;out.goal=input.goal;out.referenceReady=input.referenceReady;out.counts.rawGenerated=input.counts.rawGenerated;
 for(const auto& record:input.records) {
  Candidate c=record;
  c.reasons.erase(std::remove(c.reasons.begin(),c.reasons.end(),"shortcut_duplicate"),c.reasons.end());
  if(postOnly && c.poolIndex && c.reasons.empty())motionRejections(c.simplifiedMotion,input.records[0].simplifiedMotion,limits,"shortcut",c.reasons);
  if(c.reasons.empty()) {
   ++out.counts.motionFeasible;
   for(Id previous:out.feasible)if(sameGeometry(graph,c.simplified,out.records[previous].simplified)){c.reasons.push_back("shortcut_duplicate");break;}
   if(c.reasons.empty()){out.feasible.push_back(out.records.size());++out.counts.postShortcutUnique;}
  }
  out.records.push_back(c);
 }
 return out;
}
}
int main(int argc,char** argv){
 try {
  if(argc!=3)throw std::runtime_error("usage: route_supply_fixture_runner SNAPSHOT_DIR PROTOCOL_JSON");
  const auto began=std::chrono::steady_clock::now();const std::string dir=argv[1];
  const auto protocol=loadJson(argv[2]);const auto manifest=loadJson(dir+"/manifest.json");
  if(protocol["protocol_id"].asString()!="I2_05_SUPPLY_DIAG_V1" || !manifest["complete"].asBool())throw std::runtime_error("unsupported protocol/incomplete snapshot");
  const auto mapPath=dir+"/"+manifest["map_file"].asString();const auto plannerPath=dir+"/"+manifest["planner_file"].asString();
  if(hex64(fnv1a64(mapPath))!=manifest["map_fnv1a64"].asString() || hex64(fnv1a64(plannerPath))!=manifest["planner_fnv1a64"].asString())throw std::runtime_error("input integrity failure");
  const auto planner=loadJson(plannerPath);const auto map=loadMap(mapPath);
  if(map.version!=manifest["map_version"].asUInt64()||map.version!=planner["map_version"].asUInt64())throw std::runtime_error("snapshot version mismatch");
  PathGainVisibilityConfig cfg;cfg.horizontalFov=number(planner["sensor_model"]["horizontal_fov"]);cfg.verticalFov=number(planner["sensor_model"]["vertical_fov"]);cfg.dmax=number(planner["sensor_model"]["dmax"]);
  for(int i=0;i<3;++i){cfg.planningMin(i)=number(planner["planning_region"]["min"][i]);cfg.planningMax(i)=number(planner["planning_region"]["max"][i]);}
  std::vector<double> yaws;for(const auto& item:planner["roadmap_nodes"][0]["legacy_yaw_gains"])yaws.push_back(number(item[0]));
  RouteSnapshot view(map,cfg,yaws);
  std::vector<Eigen::Vector3d> point;Eigen::Vector3d start;for(int i=0;i<3;++i)start(i)=number(planner["vehicle"]["position"][i]);point.push_back(start);
  Id expected=0;for(const auto& n:planner["roadmap_nodes"]){if(n["id"].asUInt64()!=expected++)throw std::runtime_error("noncontiguous node IDs");Eigen::Vector3d p;for(int i=0;i<3;++i)p(i)=number(n["position"][i]);point.push_back(p);}
  for(Id i=1;i<point.size();++i)for(Id j=i+1;j<point.size();++j)if(point[i]==point[j])throw std::runtime_error("ambiguous coincident roadmap IDs");
  std::vector<Graph::Position> positions;for(const auto& p:point)positions.push_back({p(0),p(1),p(2)});
  std::vector<Edge> edges;for(const auto& e:planner["roadmap_edges"]){const Id a=e[0].asUInt64()+1,b=e[1].asUInt64()+1;
   if(a>=point.size()||b>=point.size())throw std::runtime_error("invalid edge ID");
   if(view.freeLine(point[a],point[b]))edges.push_back({a,b});
   if(view.freeLine(point[b],point[a]))edges.push_back({b,a});}
  for(Id i=1;i<point.size();++i)if((point[i]-start).norm()<=1.5 && view.freeLine(start,point[i]))edges.push_back({0,i});
  const Graph graph(positions,edges);const double initialYaw=number(planner["vehicle"]["yaw"]);
  MotionConfig motion;motion.speed=number(planner["motion_model"]["velocity"]);motion.angularSpeed=number(planner["motion_model"]["angular_velocity"]);motion.yawPenalty=number(planner["motion_model"]["yaw_penalty_weight"]);
  const auto& primary=protocol["primary"];Limits limits;limits.lengthRatio=number(primary["length_ratio"]);limits.timeRatio=number(primary["time_ratio"]);limits.extraYaw=number(primary["extra_yaw"]);limits.routesPerGoal=primary["routes_per_goal"].asUInt64();limits.globalRoutes=primary["global_selected_cap"].asUInt64();
  Json::Value output;output["schema"]="cerlab-i2-05-supply-v1";output["map_version"]=Json::UInt64(map.version);output["sequence"]=planner["planning_sequence"];output["goals"]=Json::Value(Json::arrayValue);
  std::vector<Id> goals;std::vector<Path> references;std::vector<double> goalYaws;std::vector<Candidate> preparedReferences;
  for(const auto& goal:planner["goal_candidates"]){Eigen::Vector3d p;for(int i=0;i<3;++i)p(i)=number(goal["position"][i]);Id id=point.size();for(Id i=1;i<point.size();++i)if(point[i]==p){id=i;break;}
   Json::Value row;row["position"]=goal["position"];
   if(id==point.size()){row["status"]="missing_goal";output["goals"].append(row);continue;}
   row["goal_id"]=Json::UInt64(id);Budget budget;const auto route=shortest(graph,0,id,budget);
   if(!route.found){row["status"]="unreachable";output["goals"].append(row);continue;}
   auto node=std::make_shared<PRM::Node>(p);node->numVoxels=view.gains(p,node->yawNumVoxels);const double yaw=node->getBestYaw();
   auto prepared=prepare(graph,id,{route.path},initialYaw,yaw,motion,limits,[&](Id a,Id b){return view.freeLine(point[a],point[b]);});
   if(!prepared.referenceReady)throw std::runtime_error("invalid reference");
   goals.push_back(id);references.push_back(route.path);goalYaws.push_back(yaw);preparedReferences.push_back(prepared.records[0]);
   row["status"]="reachable";row["terminal_yaw"]=yaw;row["reference"]=sequence(route.path.nodes);output["goals"].append(row);
  }
  output["reachable_goals"]=Json::UInt64(goals.size());output["roadmap_nodes"]=Json::UInt64(point.size()-1);output["validated_directed_edges"]=Json::UInt64(edges.size());
  for(const auto& kValue:protocol["diagnostic_pool_sizes"]){const Id k=kValue.asUInt64();Limits current=limits;current.poolPerGoal=k;Limits permissive=current;permissive.lengthRatio=1e12;permissive.timeRatio=1e12;permissive.extraYaw=1e12;
   std::size_t globalPops=0;std::vector<GoalPool> strictPools,postPools,freePools;Json::Value trial;trial["k"]=Json::UInt64(k);trial["goals"]=Json::Value(Json::arrayValue);
   for(Id g=0;g<goals.size();++g){Budget budget;budget.maxPops=std::min<std::size_t>(primary["heap_pops_per_goal"].asUInt64(),primary["heap_pops_global"].asUInt64()-globalPops);
    const auto routes=yen(graph,0,goals[g],k,budget,&references[g]);globalPops+=budget.pops;
    auto line=[&](Id a,Id b){return view.freeLine(point[a],point[b]);};
    const auto strict=prepare(graph,goals[g],routes.paths,initialYaw,goalYaws[g],motion,current,line,[]{return true;},&preparedReferences[g]);
    const auto loose=prepare(graph,goals[g],routes.paths,initialYaw,goalYaws[g],motion,permissive,line,[]{return true;},&preparedReferences[g]);
    const auto post=diagnosticPool(loose,graph,current,true);const auto free=diagnosticPool(loose,graph,current,false);
    strictPools.push_back(strict);postPools.push_back(post);freePools.push_back(free);
    Json::Value row;row["goal_id"]=Json::UInt64(goals[g]);row["generated"]=Json::UInt64(routes.paths.size());row["pops"]=Json::UInt64(budget.pops);row["stop"]=static_cast<int>(routes.stop);row["exhausted"]=routes.exhausted;
    row["strict_unique"]=Json::UInt64(strict.counts.postShortcutUnique);row["post_only_unique"]=Json::UInt64(post.counts.postShortcutUnique);row["no_motion_unique"]=Json::UInt64(free.counts.postShortcutUnique);
    row["motion_feasible"]=Json::UInt64(strict.counts.motionFeasible);row["candidates"]=Json::Value(Json::arrayValue);
    for(Id i=0;i<strict.records.size();++i){const auto& c=strict.records[i];Json::Value record;record["pool_index"]=Json::UInt64(i);record["raw"]=sequence(c.raw);record["shortcut"]=sequence(c.simplified);record["raw_motion"]=motionJson(c.rawMotion);record["shortcut_motion"]=motionJson(c.simplifiedMotion);record["rejections"]=Json::Value(Json::arrayValue);for(const auto& r:c.reasons)record["rejections"].append(r);row["candidates"].append(record);}
    trial["goals"].append(row);
   }
   for(const auto& name:{"strict","post_only","no_motion"}){const auto& pools=std::string(name)=="strict"?strictPools:(std::string(name)=="post_only"?postPools:freePools);
    const auto generic=select(pools,Mode::GENERIC_DISTANCE,current);const auto diverse=select(pools,Mode::GEOMETRIC_DIVERSE,current);
    trial[name]["generic_selected_count"]=Json::UInt64(generic.routes.size());trial[name]["diverse_selected_count"]=Json::UInt64(diverse.routes.size());
    trial[name]["selected_extra_count"]=Json::UInt64(generic.routes.size()-goals.size());
    bool changed=false;for(Id i=0;i<generic.routes.size();++i)if(generic.routes[i].goal!=diverse.routes[i].goal||generic.routes[i].poolIndex!=diverse.routes[i].poolIndex)changed=true;
    trial[name]["generic_diverse_selection_changed"]=changed;
    Json::Value chosen(Json::arrayValue);for(const auto& c:generic.routes){Json::Value item;item["goal"]=Json::UInt64(c.goal);item["pool_index"]=Json::UInt64(c.poolIndex);chosen.append(item);}trial[name]["generic_choices"]=chosen;
   }
   trial["alternative_pops"]=Json::UInt64(globalPops);output["trials"].append(trial);
  }
  output["elapsed_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  Json::StreamWriterBuilder writer;writer["indentation"]="";std::cout<<Json::writeString(writer,output)<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
