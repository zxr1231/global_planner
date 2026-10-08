#include <gtest/gtest.h>
#include <global_planner/dep.h>
#include <global_planner/PRMAstar.h>
#include <jsoncpp/json/json.h>
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
 static void snapshot(DEP& p,bool enable){p.routeSnapshot_=enable?std::make_shared<const mapManager::OccupancyMapSnapshot>(p.map_->captureSnapshot()):nullptr;}
 static bool generate(DEP& p,const std::string& mode){p.routeMode_=mode;return p.findRouteControlCandidates();}
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
 std::vector<Eigen::Vector3d> points={{-4.1,0,1},{-1.1,.13,.91},{2.2,0,1},{3.99,0,1},{4,0,1}};
 for(const auto& a:points){EXPECT_EQ(map->isInflatedFree(a),frozen.free(a));EXPECT_EQ(map->isUnknown(a),frozen.unknown(a));
  for(const auto& b:points){EXPECT_EQ(map->isInflatedFreeLine(a,b),frozen.freeLine(a,b));EXPECT_EQ(map->isInflatedOccupiedLine(a,b),frozen.occluded(a,b));}}
 map->obstacle(Eigen::Vector3d(0,0,1));EXPECT_TRUE(frozen.free(Eigen::Vector3d(0,0,1)));EXPECT_FALSE(map->isInflatedFree(Eigen::Vector3d(0,0,1)));
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
 }
}
int main(int argc,char** argv){ros::init(argc,argv,"i2_snapshot_checks",ros::init_options::AnonymousName);testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
