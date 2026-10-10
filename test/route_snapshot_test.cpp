#include <gtest/gtest.h>
#include <global_planner/dep.h>
#include <global_planner/PRMAstar.h>
#include <jsoncpp/json/json.h>
#include <thread>
#include <atomic>
class FrozenTestMap: public mapManager::occMap {
 public:
 FrozenTestMap() {
  mapRes_=.25;mapSizeMin_=Eigen::Vector3d(-4,-4,0);mapSizeMax_=Eigen::Vector3d(4,4,2);
  mapVoxelMin_=Eigen::Vector3i::Zero();mapVoxelMax_=Eigen::Vector3i(32,32,8);
  pMinLog_=-2;pOccLog_=1;mapVersion_=42;
  occupancy_.assign(32*32*8,-2);occupancyInflated_.assign(32*32*8,false);
  for(int x=24;x<32;++x)for(int y=0;y<32;++y)for(int z=0;z<8;++z)
   occupancy_[indexToAddress(Eigen::Vector3i(x,y,z))]=-3;
 }
 void obstacle(const Eigen::Vector3d& p){occupancyInflated_[posToAddress(p)]=true;++mapVersion_;}
};
namespace globalPlanner {
struct RouteControlTestAccess {
 static void configure(DEP& p,const std::shared_ptr<FrozenTestMap>& map) {
  p.map_=map;p.currYaw_=0;p.position_=Eigen::Vector3d(-1.5,0,1);p.maxConnectDist_=.6;
  p.globalRegionMin_=Eigen::Vector3d(-3,-3,.7);p.globalRegionMax_=Eigen::Vector3d(3,3,1.2);
  p.horizontalFOV_=M_PI/2;p.verticalFOV_=M_PI/2;p.dmax_=2;p.vel_=1;p.angularVel_=1;p.yawPenaltyWeight_=1;
  p.prmNodeVec_.clear();p.goalCandidates_.clear();std::shared_ptr<PRM::Node> prev;
  for(double x:{-1.0,0.0,1.0}) {
   auto n=std::make_shared<PRM::Node>(Eigen::Vector3d(x,0,1));p.prmNodeVec_.insert(n);
   if(prev){prev->adjNodes.insert(n);n->adjNodes.insert(prev);}prev=n;
  }
  p.goalCandidates_.push_back(prev);
 }
 static std::shared_ptr<PRM::Node> goal(DEP& p){return p.goalCandidates_.front();}
 static const auto& gainSources(DEP& p){return p.routeGainSources_;}
 static void snapshot(DEP& p,bool enable){p.routeSnapshot_=enable?std::make_shared<const mapManager::OccupancyMapSnapshot>(p.map_->captureSnapshot()):nullptr;}
 static bool generate(DEP& p,const std::string& mode){
  p.routeMode_=mode;p.routePoseMailbox_.update(p.position_,p.currYaw_,1.0);return p.findRouteControlCandidates();
 }
 static const auto& paths(DEP& p){return p.candidatePaths_;}
 static const auto& metrics(DEP& p){return p.candidateLegacyMetrics_;}
 static const auto& yaws(DEP& p){return p.yaws_;}
 static PathGainVisibilityConfig config(DEP& p){PathGainVisibilityConfig c;c.horizontalFov=p.horizontalFOV_;c.verticalFov=p.verticalFOV_;c.dmax=p.dmax_;c.planningMin=p.globalRegionMin_;c.planningMax=p.globalRegionMax_;return c;}
 static void shadow(DEP& p){p.pathGainMode_=PathGainMode::UNIQUE_SHADOW;p.evaluateUniquePathGain(p.candidatePaths_);}
 static uint64_t version(DEP& p){return p.routeSnapshot_->version;}
 static std::vector<std::vector<std::shared_ptr<PRM::Node>>> legacy(DEP& p){std::vector<std::vector<std::shared_ptr<PRM::Node>>> paths;p.findCandidatePath(p.goalCandidates_,paths);return paths;}
};
}
TEST(RouteSnapshot, CollisionUnknownBoundsAndFrozenMutationMatchRealMap) {
 auto map=std::make_shared<FrozenTestMap>();auto snapshot=map->captureSnapshot();
 globalPlanner::PathGainVisibilityConfig c;c.horizontalFov=c.verticalFov=M_PI/2;c.dmax=2;
 c.planningMin=Eigen::Vector3d(-3,-3,.7);c.planningMax=Eigen::Vector3d(3,3,1.2);
 globalPlanner::RouteSnapshot frozen(snapshot,c,{0,M_PI});
 globalPlanner::FrozenRouteMap compatibility(snapshot);
 std::vector<Eigen::Vector3d> points={{-4.1,0,1},{-1.1,.13,.91},{2.2,0,1},{3.99,0,1},{4,0,1}};
 for(const auto& a:points){EXPECT_EQ(map->isInflatedFree(a),frozen.free(a));EXPECT_EQ(compatibility.isInflatedFree(a),frozen.free(a));EXPECT_EQ(map->isUnknown(a),frozen.unknown(a));
  for(const auto& b:points){EXPECT_EQ(map->isInflatedFreeLine(a,b),frozen.freeLine(a,b));EXPECT_EQ(map->isInflatedOccupiedLine(a,b),frozen.occluded(a,b));}}
 map->obstacle(Eigen::Vector3d(0,0,1));EXPECT_TRUE(compatibility.isInflatedFree(Eigen::Vector3d(0,0,1)));EXPECT_TRUE(frozen.free(Eigen::Vector3d(0,0,1)));EXPECT_FALSE(map->isInflatedFree(Eigen::Vector3d(0,0,1)));
}
TEST(RouteSnapshot, LegacyGainAndScoringMatchOnFrozenOffCentreInputs) {
 ros::NodeHandle nh;globalPlanner::DEP planner(nh);auto map=std::make_shared<FrozenTestMap>();
 using A=globalPlanner::RouteControlTestAccess;A::configure(planner,map);
 for(double x:{-1.5,-.113,1.031,2.21}) {
  auto node=std::make_shared<PRM::Node>(Eigen::Vector3d(x,.133,.901));std::unordered_map<double,int> live,frozen;
  A::snapshot(planner,false);const int count=planner.calculateUnknown(node,live);
  A::snapshot(planner,true);EXPECT_EQ(count,planner.calculateUnknown(node,frozen));EXPECT_EQ(live,frozen);
 }
 A::snapshot(planner,false);auto paths=A::legacy(planner);std::vector<std::shared_ptr<PRM::Node>> best;
 planner.findBestPath(paths,best);auto expected=A::metrics(planner);
 ASSERT_TRUE(A::generate(planner,"distance_single"));planner.findBestPath(A::paths(planner),best);
 ASSERT_EQ(expected.size(),A::metrics(planner).size());
 for(size_t i=0;i<expected.size();++i){const auto& m=A::metrics(planner)[i];EXPECT_EQ(expected[i].gain,m.gain);EXPECT_DOUBLE_EQ(expected[i].score,m.score);EXPECT_DOUBLE_EQ(expected[i].yawDistance,m.yawDistance);}
 A::shadow(planner);EXPECT_EQ(A::version(planner),planner.getLastPlanningMetrics().uniqueMapVersion);
}
TEST(RouteSnapshot, AllControlModesReturnSafeRoutesAndAuditablePool) {
 ros::NodeHandle nh;globalPlanner::DEP planner(nh);auto map=std::make_shared<FrozenTestMap>();
 using A=globalPlanner::RouteControlTestAccess;A::configure(planner,map);
 for(const std::string mode:{"distance_single","generic_k_shortest","geometric_diverse"}) {
  ASSERT_TRUE(A::generate(planner,mode));ASSERT_FALSE(A::paths(planner).empty());
  for(const auto& p:A::paths(planner))for(size_t i=1;i<p.size();++i)EXPECT_TRUE(map->isInflatedFreeLine(p[i-1]->pos,p[i]->pos));
  Json::Value log;Json::CharReaderBuilder reader;std::istringstream in(planner.getLastPlanningMetrics().routeControlJson);std::string error;
  ASSERT_TRUE(Json::parseFromStream(reader,in,&log,&error));EXPECT_EQ("ready",log["status"].asString());
  EXPECT_EQ(42u,log["map_version"].asUInt64());EXPECT_EQ(1u,log["selected_count"].asUInt64());
  for(const auto& entry:log["astar_comparisons"]){EXPECT_TRUE(entry["snapshot_matches_comparison_map"].asBool());EXPECT_NEAR(0,entry["excess_length"].asDouble(),1e-10);}
 }
}
TEST(RouteSnapshot, PoseMailboxReturnsCoherentPairsAndRejectsInvalidSamples) {
 globalPlanner::RoutePoseMailbox mailbox;EXPECT_FALSE(mailbox.capture().valid);
 std::atomic<bool> done{false};std::atomic<int> bad{0};
 std::thread writer([&]{for(int i=0;i<20000;++i){double x=i%2;mailbox.update(Eigen::Vector3d(x,2*x,3*x),x,4*x);}done=true;});
 do {const auto p=mailbox.capture();if(p.valid && (p.position.x()!=p.yaw || p.position.y()!=2*p.yaw || p.stamp!=4*p.yaw))++bad;} while(!done);
 writer.join();EXPECT_EQ(0,bad.load());EXPECT_EQ(20000u,mailbox.capture().sequence);
 mailbox.update(Eigen::Vector3d::Zero(),std::numeric_limits<double>::quiet_NaN(),0);EXPECT_FALSE(mailbox.capture().valid);
}
TEST(RouteSnapshot, ScoringAndRouteStartStayFixedDuringOdometryUpdates) {
 ros::NodeHandle nh;globalPlanner::DEP planner(nh);auto map=std::make_shared<FrozenTestMap>();
 using A=globalPlanner::RouteControlTestAccess;A::configure(planner,map);ASSERT_TRUE(A::generate(planner,"distance_single"));
 std::vector<std::shared_ptr<PRM::Node>> best;planner.findBestPath(A::paths(planner),best);const auto expected=A::metrics(planner);
 const auto start=A::paths(planner).front().front()->pos;
 std::atomic<bool> done{false};std::thread writer([&]{
  while(!done){auto odom=boost::make_shared<nav_msgs::Odometry>();odom->pose.pose.position.x=2;odom->pose.pose.position.z=1;
   odom->pose.pose.orientation=globalPlanner::quaternion_from_rpy(0,0,1.0);planner.odomCB(odom);}
 });
 for(int i=0;i<20;++i){planner.findBestPath(A::paths(planner),best);EXPECT_DOUBLE_EQ(expected[0].score,A::metrics(planner)[0].score);EXPECT_DOUBLE_EQ(expected[0].yawDistance,A::metrics(planner)[0].yawDistance);EXPECT_EQ(start,A::paths(planner).front().front()->pos);}
 done=true;writer.join();
 // Historical path still uses its live-yaw semantics when no new-control snapshot is active.
 A::snapshot(planner,false);planner.findBestPath(A::paths(planner),best);EXPECT_NE(expected[0].yawDistance,A::metrics(planner)[0].yawDistance);
}
TEST(RouteSnapshot, ScoredGoalGainMustFeedNextRoadmapPrefilter) {
 ros::NodeHandle nh;globalPlanner::DEP planner(nh);auto map=std::make_shared<FrozenTestMap>();
 using A=globalPlanner::RouteControlTestAccess;A::configure(planner,map);
 auto original=A::goal(planner);original->numVoxels=999999;
 auto legacy=A::legacy(planner);std::vector<std::shared_ptr<PRM::Node>> best;
 planner.findBestPath(legacy,best);const auto expected=original->numVoxels;
 ASSERT_LT(expected,999999);
 for(const std::string mode:{"distance_single","generic_k_shortest","geometric_diverse"}) {
  original->numVoxels=999999;original->g=123;original->f=456;
  const auto parent=original->parent;const auto adjacency=original->adjNodes;
  ASSERT_TRUE(A::generate(planner,mode));
  EXPECT_EQ(999999,original->numVoxels); // construction must not refresh unscored nodes
  planner.findBestPath(A::paths(planner),best);
  EXPECT_EQ(expected,original->numVoxels) << "detached scoring lost live-roadmap gain feedback";
  EXPECT_EQ(A::paths(planner).front().back()->yawNumVoxels,original->yawNumVoxels);
  EXPECT_DOUBLE_EQ(123,original->g);EXPECT_DOUBLE_EQ(456,original->f);
  EXPECT_EQ(parent,original->parent);EXPECT_EQ(adjacency,original->adjNodes);
 }
}
TEST(RouteSnapshot, IntermediateFeedbackLeavesUnscoredNodesUntouched) {
 ros::NodeHandle nh;globalPlanner::DEP planner(nh);auto map=std::make_shared<FrozenTestMap>();
 using A=globalPlanner::RouteControlTestAccess;A::configure(planner,map);
 ASSERT_TRUE(A::generate(planner,"distance_single"));
 std::shared_ptr<PRM::Node> middle,liveMiddle,unscored;
 for(const auto& pair:A::gainSources(planner)) {
  pair.second->numVoxels=999999;
  if(pair.first->pos.x()==0){middle=pair.first;liveMiddle=pair.second;}
  if(pair.first->pos.x()==-1)unscored=pair.second;
 }
 ASSERT_TRUE(middle);ASSERT_TRUE(unscored);
 const auto& originalPath=A::paths(planner).front();
 std::vector<std::vector<std::shared_ptr<PRM::Node>>> paths{{originalPath.front(),middle,originalPath.back()}};
 std::vector<std::shared_ptr<PRM::Node>> best;planner.findBestPath(paths,best);
 EXPECT_LT(liveMiddle->numVoxels,999999);EXPECT_EQ(middle->numVoxels,liveMiddle->numVoxels);
 EXPECT_EQ(middle->yawNumVoxels,liveMiddle->yawNumVoxels);EXPECT_EQ(999999,unscored->numVoxels);
 // No snapshot means historical fallback: a stale association must have no effect.
 A::snapshot(planner,false);liveMiddle->numVoxels=999999;
 planner.findBestPath(paths,best);EXPECT_EQ(999999,liveMiddle->numVoxels);
}
int main(int argc,char** argv){ros::init(argc,argv,"i2_snapshot_checks",ros::init_options::AnonymousName);testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
