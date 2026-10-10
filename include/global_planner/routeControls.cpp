#include <global_planner/dep.h>
#include <global_planner/PRMAstar.h>
#include <jsoncpp/json/json.h>
#include <map>
#include <sstream>
#include <iomanip>

namespace globalPlanner {
void DEP::syncRouteNodeGain(const std::shared_ptr<PRM::Node>& node) {
 if (!routeSnapshot_) return;
 const auto source=routeGainSources_.find(node);
 if (source==routeGainSources_.end()) return; // synthetic start / historical node
 source->second->numVoxels=node->numVoxels;
 source->second->yawNumVoxels=node->yawNumVoxels;
}

bool DEP::findRouteControlCandidates() {
 using namespace routeSearch;
 using namespace routeCandidates;
 using Clock=std::chrono::steady_clock;
 const auto begun=Clock::now();
 routeGainSources_.clear();
 Json::Value log(Json::objectValue);
 log["protocol"]="I2_GENERIC_ROUTES_V1";log["protocol_sha256"]="9136d601b964fc273565e381c91da6075c33b7b47a4269d6664714df281d4f9d";log["mode"]=this->routeMode_;
 log["gain_feedback_policy"]="scored_nodes_to_roadmap_v1";
 log["status"]="building";log["fallback"]=false;
 auto finish=[&](const std::string& status,bool fallback) {
  log["status"]=status;log["fallback"]=fallback;
  log["adapter_total_ms"]=std::chrono::duration<double,std::milli>(Clock::now()-begun).count();
  Json::StreamWriterBuilder writer;writer["indentation"]="";
  this->lastPlanningMetrics_.routeControlJson=Json::writeString(writer,log);
 };
 try {
  routeStartPose_=routePoseMailbox_.capture();
  if(!routeStartPose_.valid)throw std::runtime_error("route pose unavailable or nonfinite");
  const Eigen::Vector3d startPosition=routeStartPose_.position;
  const double startYaw=routeStartPose_.yaw;
  log["pose_sequence"]=Json::UInt64(routeStartPose_.sequence);log["pose_stamp"]=routeStartPose_.stamp;
  log["pose_context_schema"]=1;
  this->routeSnapshot_=std::make_shared<const mapManager::OccupancyMapSnapshot>(this->map_->captureSnapshot());
  const auto& snapshot=*this->routeSnapshot_;log["map_version"]=Json::UInt64(snapshot.version);
  PathGainVisibilityConfig visibility;
  visibility.horizontalFov=horizontalFOV_;visibility.verticalFov=verticalFOV_;visibility.dmax=dmax_;
  visibility.planningMin=globalRegionMin_;visibility.planningMax=globalRegionMax_;
  RouteSnapshot view(snapshot,visibility,yaws_);
  std::vector<std::shared_ptr<PRM::Node>> live(prmNodeVec_.begin(),prmNodeVec_.end());
  for(const auto& node:live)if(!node->pos.allFinite())throw std::invalid_argument("nonfinite roadmap position");
  std::sort(live.begin(),live.end(),[](const auto& a,const auto& b) {
   for(int i=0;i<3;++i)if(a->pos(i)!=b->pos(i))return a->pos(i)<b->pos(i);
   return false;
  });
  for(std::size_t i=1;i<live.size();++i)
   if(live[i]->pos==live[i-1]->pos)throw std::invalid_argument("duplicate roadmap position needs persistent identity");
  std::vector<std::shared_ptr<PRM::Node>> detached{std::make_shared<PRM::Node>(startPosition)};
  struct ClearAdjacency {
   std::vector<std::shared_ptr<PRM::Node>>& nodes;
   ~ClearAdjacency(){for(const auto& node:nodes)node->adjNodes.clear();}
  } cleanup{detached};
  std::map<std::shared_ptr<PRM::Node>,Id> ids;
  std::vector<Graph::Position> positions{{startPosition(0),startPosition(1),startPosition(2)}};
  for(const auto& node:live) {
   ids[node]=detached.size();detached.push_back(std::make_shared<PRM::Node>(node->pos));
   routeGainSources_.emplace(detached.back(),node);
   positions.push_back({node->pos(0),node->pos(1),node->pos(2)});
  }
  std::vector<Edge> edges;
  for(const auto& node:live) {
   const Id id=ids.at(node);
   if((node->pos-startPosition).norm()<=maxConnectDist_ && view.freeLine(startPosition,node->pos))edges.push_back({0,id});
   for(const auto& next:node->adjNodes)
    if(ids.count(next) && view.freeLine(node->pos,next->pos))edges.push_back({id,ids.at(next)});
  }
  const Graph graph(positions,edges);
  Json::Value nodes(Json::arrayValue),arcs(Json::arrayValue);
  for(const auto& p:positions){Json::Value point(Json::arrayValue);for(double v:p)point.append(v);nodes.append(point);}
  for(Id i=0;i<graph.size();++i)for(const auto& edge:graph.neighbors(i)) {
   Json::Value pair(Json::arrayValue);pair.append(Json::UInt64(i));pair.append(Json::UInt64(edge.to));arcs.append(pair);
  }
  log["graph_nodes"]=nodes;log["graph_edges"]=arcs;log["start_yaw"]=startYaw;
  Limits limits;MotionConfig motion;motion.speed=vel_;motion.angularSpeed=angularVel_;motion.yawPenalty=yawPenaltyWeight_;
  log["pool_per_goal"]=6;log["routes_per_goal"]=3;log["global_cap"]=30;
  log["length_ratio"]=limits.lengthRatio;log["time_ratio"]=limits.timeRatio;log["extra_yaw"]=limits.extraYaw;
  std::vector<Id> goals;std::vector<Path> references;std::vector<double> terminalYaws;
  Json::Value goalLog(Json::arrayValue);Json::UInt64 referencePops=0;
  for(const auto& goal:goalCandidates_) {
   Json::Value entry;const auto found=ids.find(goal);
   if(found==ids.end()){entry["status"]="missing_goal";goalLog.append(entry);continue;}
   const Id id=found->second;entry["goal_id"]=Json::UInt64(id);
   Budget budget;const auto route=shortest(graph,0,id,budget);referencePops+=budget.pops;
   if(!route.found){entry["status"]="unreachable";goalLog.append(entry);continue;}
   detached[id]->numVoxels=view.gains(detached[id]->pos,detached[id]->yawNumVoxels);
   const double yaw=detached[id]->getBestYaw();
   goals.push_back(id);references.push_back(route.path);terminalYaws.push_back(yaw);
   entry["status"]="reachable";entry["terminal_yaw"]=yaw;entry["reference_cost"]=route.path.cost;goalLog.append(entry);
  }
  log["goal_set"]=goalLog;log["reference_pops"]=referencePops;
  if(goals.empty()){finish("no_reachable_prefiltered_goal",true);return false;}
  // Planning uses a worker thread: live mapping can change during this function.
  // Compare the unchanged A* on a frozen map wrapper, never a live read-through map.
  auto comparisonMap=std::make_shared<FrozenRouteMap>(snapshot);
  for(Id i=0;i<graph.size();++i)for(const auto& edge:graph.neighbors(i))detached[i]->adjNodes.insert(detached[edge.to]);
  for(Id g=0;g<goals.size();++g) {
   Json::Value comparison;comparison["goal_id"]=Json::UInt64(goals[g]);
   const uint64_t before=comparisonMap->getMapVersion();
   auto legacy=PRM::AStar(roadmap_,detached[0],detached[goals[g]],comparisonMap);
   const uint64_t after=comparisonMap->getMapVersion();
   comparison["snapshot_matches_comparison_map"]=(before==snapshot.version && after==before);
   comparison["found"]=!legacy.empty();comparison["reference_cost"]=references[g].cost;
   if(!legacy.empty()) {
    double cost=0;for(size_t i=1;i<legacy.size();++i)cost+=(legacy[i]->pos-legacy[i-1]->pos).norm();
    comparison["astar_cost"]=cost;comparison["excess_length"]=cost-references[g].cost;
   }
   log["astar_comparisons"].append(comparison);
  }
  comparisonMap.reset();
  // Avoid shared_ptr adjacency cycles in temporary nodes retained by output paths.
  for(const auto& node:detached)node->adjNodes.clear();
  std::vector<Candidate> preparedReferences;
  for(Id g=0;g<goals.size();++g) {
   auto p=prepare(graph,goals[g],{references[g]},startYaw,terminalYaws[g],motion,limits,
      [&](Id a,Id b){return view.freeLine(detached[a]->pos,detached[b]->pos);});
   if(!p.referenceReady) throw std::runtime_error("reference validation failed");
   preparedReferences.push_back(p.records.front());
  }
  log["mandatory_stage_ms"]=std::chrono::duration<double,std::milli>(Clock::now()-begun).count();
  const auto alternativesStarted=Clock::now();const auto deadline=alternativesStarted+std::chrono::milliseconds(50);
  auto proceed=[&]{return Clock::now()<deadline;};
  std::size_t globalPops=0;std::vector<GoalPool> pools;
  for(Id g=0;g<goals.size();++g) {
   std::vector<Path> raw{references[g]};
   if(routeMode_!="distance_single") {
    Budget alternative;alternative.maxPops=std::min<std::size_t>(10000,100000-globalPops);
    alternative.hasDeadline=true;alternative.deadline=deadline;
    const auto result=yen(graph,0,goals[g],6,alternative,&references[g]);
    raw=result.paths;globalPops+=alternative.pops;
    log["search_cutoffs"].append(static_cast<int>(result.stop));
   }
   auto pool=prepare(graph,goals[g],raw,startYaw,terminalYaws[g],motion,limits,
    [&](Id a,Id b){return view.freeLine(detached[a]->pos,detached[b]->pos);},proceed,&preparedReferences[g]);
   pools.push_back(std::move(pool));
  }
  Limits selectionLimits=limits;if(routeMode_=="distance_single")selectionLimits.routesPerGoal=1;
  const auto selected=select(pools,routeMode_=="geometric_diverse"?Mode::GEOMETRIC_DIVERSE:Mode::GENERIC_DISTANCE,selectionLimits,proceed);
  log["alternative_pops"]=Json::UInt64(globalPops);log["selection_cutoff"]=selected.cutoff;
  log["alternative_elapsed_ms"]=std::chrono::duration<double,std::milli>(Clock::now()-alternativesStarted).count();
  Json::Value poolLog(Json::arrayValue);std::size_t rawCount=0,feasibleCount=0,distinctCount=0;
  auto sequence=[](const Sequence& s){Json::Value a(Json::arrayValue);for(Id id:s)a.append(Json::UInt64(id));return a;};
  for(const auto& pool:pools) {
   Json::Value p;p["goal_id"]=Json::UInt64(pool.goal);p["cutoff"]=pool.cutoff;
   p["raw_generated"]=Json::UInt64(pool.counts.rawGenerated);rawCount+=pool.counts.rawGenerated;
   p["motion_feasible"]=Json::UInt64(pool.counts.motionFeasible);feasibleCount+=pool.counts.motionFeasible;
   p["post_shortcut_unique"]=Json::UInt64(pool.counts.postShortcutUnique);distinctCount+=pool.counts.postShortcutUnique;
   p["candidates"]=Json::Value(Json::arrayValue);
   for(const auto& candidate:pool.records) {
    Json::Value c;c["pool_index"]=Json::UInt64(candidate.poolIndex);c["raw"]=sequence(candidate.raw);c["shortcut"]=sequence(candidate.simplified);
    c["raw_length"]=candidate.rawMotion.length;c["raw_time"]=candidate.rawMotion.time;c["raw_yaw"]=candidate.rawMotion.yaw;
    c["shortcut_length"]=candidate.simplifiedMotion.length;c["shortcut_time"]=candidate.simplifiedMotion.time;c["shortcut_yaw"]=candidate.simplifiedMotion.yaw;
    c["rejections"]=Json::Value(Json::arrayValue);for(const auto& r:candidate.reasons)c["rejections"].append(r);p["candidates"].append(c);
   }
   poolLog.append(p);
  }
  log["pools"]=poolLog;log["raw_generated"]=Json::UInt64(rawCount);
  log["motion_feasible"]=Json::UInt64(feasibleCount);log["post_shortcut_unique"]=Json::UInt64(distinctCount);
  candidatePaths_.clear();candidateRawPaths_.clear();lastRecoveryUsed_=false;
  for(const auto& choice:selected.routes) {
   const auto it=std::find_if(pools.begin(),pools.end(),[&](const GoalPool& p){return p.goal==choice.goal;});
   const auto& record=it->records.at(choice.poolIndex);
   std::vector<std::shared_ptr<PRM::Node>> raw,path;
   for(Id id:record.raw)raw.push_back(detached[id]);
   for(Id id:record.simplified)path.push_back(detached[id]);
   candidateRawPaths_.push_back(raw);candidatePaths_.push_back(path);
   Json::Value entry;entry["goal_id"]=Json::UInt64(choice.goal);entry["pool_index"]=Json::UInt64(choice.poolIndex);log["selected_routes"].append(entry);
  }
  log["selected_count"]=Json::UInt64(candidatePaths_.size());
  finish(candidatePaths_.empty()?"empty_after_constraints":"ready",candidatePaths_.empty());
  return !candidatePaths_.empty();
 } catch(const std::exception& error) {
  // Error text is serialized by JsonCpp, never interpolated as unescaped JSON.
  log["error"]=error.what();finish("adapter_error",true);
  ROS_ERROR("[DEP][I2] Route adapter failed, using historical fallback: %s",error.what());return false;
 }
}
}
