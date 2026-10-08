#include <gtest/gtest.h>
#include <global_planner/routeCandidates.h>
using namespace globalPlanner::routeCandidates;
static Graph fixture() {
  return Graph({{0,0,0},{1,0,0},{2,0,0},{2,.1,0},{1.5,1,0},{3,0,0}},
      {{0,1},{1,2},{2,5},{1,3},{3,5},{0,4},{4,5}});
}
static std::vector<Path> routes(const Graph& graph) {
  Budget b; return yen(graph,0,5,6,b).paths;
}
static LineCheck adjacency(const Graph& graph) {
  return [&graph](Id a,Id b) {
    for(const auto& arc:graph.neighbors(a)) if(arc.to==b) return true;
    return false;
  };
}
static bool reason(const Candidate& c,const std::string& why) {
  return std::find(c.reasons.begin(),c.reasons.end(),why)!=c.reasons.end();
}
TEST(RouteCandidates, SamePoolDifferentGeometricChoiceNoMutation) {
  auto graph=fixture(); auto raw=routes(graph); Limits limits; limits.routesPerGoal=2; limits.extraYaw=10;
  MotionConfig motion; motion.yawPenalty=0;
  auto pool=prepare(graph,5,raw,0,0,motion,limits,adjacency(graph));
  ASSERT_EQ(3u,pool.feasible.size());
  auto distance=select({pool},Mode::GENERIC_DISTANCE,limits);
  auto diverse=select({pool},Mode::GEOMETRIC_DIVERSE,limits);
  ASSERT_EQ(2u,distance.routes.size()); ASSERT_EQ(2u,diverse.routes.size());
  EXPECT_EQ(0u,distance.routes[0].poolIndex); EXPECT_EQ(0u,diverse.routes[0].poolIndex);
  EXPECT_EQ(1u,distance.routes[1].poolIndex); EXPECT_EQ(2u,diverse.routes[1].poolIndex);
  EXPECT_EQ(raw[1].nodes,pool.records[1].raw);
  EXPECT_EQ(3u,pool.counts.rawGenerated); EXPECT_EQ(3u,pool.counts.motionFeasible);
  EXPECT_EQ(3u,pool.counts.postShortcutUnique);
  EXPECT_DOUBLE_EQ(1.0,jaccardDistance(pool.records[0].simplified,pool.records[2].simplified));
  EXPECT_LT(jaccardDistance(pool.records[0].simplified,pool.records[1].simplified),1.0);
}
TEST(RouteCandidates, ShortcutCollapseAndRawDuplicatesHaveHonestCounts) {
  auto graph=fixture(); auto raw=routes(graph); raw.push_back(raw[1]);
  Limits limits; limits.extraYaw=10; MotionConfig motion; motion.yawPenalty=0;
  auto pool=prepare(graph,5,raw,0,0,motion,limits,[](Id,Id){return true;});
  EXPECT_EQ(4u,pool.counts.rawGenerated); EXPECT_EQ(3u,pool.counts.motionFeasible);
  EXPECT_EQ(1u,pool.counts.postShortcutUnique); EXPECT_TRUE(reason(pool.records[1],"shortcut_duplicate"));
  EXPECT_TRUE(reason(pool.records[3],"raw_duplicate"));
  auto selected=select({pool},Mode::GEOMETRIC_DIVERSE,limits);
  EXPECT_EQ(1u,selected.routes.size()); EXPECT_EQ(1u,selected.selectedPerGoal[0].second);
}
TEST(RouteCandidates, IndependentLengthTimeYawAndSimplifiedConstraints) {
  auto graph=fixture(); auto raw=routes(graph); Limits limits; MotionConfig motion;
  limits.lengthRatio=1; limits.timeRatio=100; limits.extraYaw=100;
  auto length=prepare(graph,5,raw,0,0,motion,limits,adjacency(graph));
  EXPECT_TRUE(reason(length.records[1],"raw_length")); EXPECT_TRUE(reason(length.records[1],"shortcut_length"));
  limits.lengthRatio=100; limits.timeRatio=1;
  auto time=prepare(graph,5,raw,0,0,motion,limits,adjacency(graph));
  EXPECT_TRUE(reason(time.records[1],"raw_time")); EXPECT_TRUE(reason(time.records[1],"shortcut_time"));
  limits.timeRatio=100; limits.extraYaw=0;
  auto yaw=prepare(graph,5,raw,0,0,motion,limits,adjacency(graph));
  EXPECT_TRUE(reason(yaw.records[1],"raw_yaw")); EXPECT_TRUE(reason(yaw.records[1],"shortcut_yaw"));
  // Raw constraint passes, but shortening only the reference tightens the shortcut constraint.
  Graph detour({{0,0,0},{1,1,0},{2,0,0},{1.5,1.25,0},{3,0,0}},
      {{0,1},{1,2},{2,4},{0,3},{3,4}});
  std::vector<Path> pool={{{0,1,2,4},0},{{0,3,4},0}};
  limits.lengthRatio=1.25; limits.extraYaw=100; motion.yawPenalty=0;
  auto simplified=prepare(detour,4,pool,0,0,motion,limits,[&](Id a,Id b){
    return (a==0 && b==2) || adjacency(detour)(a,b);
  });
  EXPECT_FALSE(reason(simplified.records[1],"raw_length"));
  EXPECT_TRUE(reason(simplified.records[1],"shortcut_length"));

}
TEST(RouteCandidates, DirectedJaccardAndPositionTolerance) {
  EXPECT_DOUBLE_EQ(1,jaccardDistance({0,1,2},{2,1,0}));
  EXPECT_DOUBLE_EQ(0,jaccardDistance({0},{1}));
  Graph graph({{0,0,0},{1,0,0},{1+5e-7,0,0},{1+2e-6,0,0}},{});
  EXPECT_TRUE(sameGeometry(graph,{0,1},{0,2})); EXPECT_FALSE(sameGeometry(graph,{0,1},{0,3}));
  EXPECT_FALSE(sameGeometry(graph,{0,1},{0,1,2}));
}
TEST(RouteCandidates, StableRoundRobinCapsAndMandatoryReferences) {
  auto graph=fixture(); Limits limits; limits.extraYaw=10; limits.globalRoutes=3;
  MotionConfig motion; motion.yawPenalty=0;
  auto a=prepare(graph,5,routes(graph),0,0,motion,limits,adjacency(graph));
  // Synthetic exported goal labels only for selection order test, geometry is already prepared.
  auto b=a; b.goal=10;
  auto selection=select({b,a},Mode::GENERIC_DISTANCE,limits);
  ASSERT_EQ(3u,selection.routes.size());
  EXPECT_EQ(5u,selection.routes[0].goal); EXPECT_EQ(10u,selection.routes[1].goal);
  EXPECT_EQ(5u,selection.routes[2].goal); EXPECT_EQ(0u,selection.routes[1].poolIndex);
  auto repeat=select({a,b},Mode::GENERIC_DISTANCE,limits);
  for(Id i=0;i<selection.routes.size();++i) {
    EXPECT_EQ(selection.routes[i].goal,repeat.routes[i].goal);
    EXPECT_EQ(selection.routes[i].poolIndex,repeat.routes[i].poolIndex);
  }
  limits.globalRoutes=1; EXPECT_THROW(select({a,b},Mode::GENERIC_DISTANCE,limits),std::invalid_argument);
  EXPECT_THROW(select({a,a},Mode::GENERIC_DISTANCE,limits),std::invalid_argument);
}
TEST(RouteCandidates, InvalidReferenceCutoffZeroReferenceAndCollision) {
  auto graph=fixture(); auto raw=routes(graph); Limits limits; MotionConfig motion;
  auto cutoff=prepare(graph,5,raw,0,0,motion,limits,adjacency(graph),[]{return false;});
  EXPECT_TRUE(cutoff.referenceReady); EXPECT_TRUE(cutoff.cutoff);
  EXPECT_TRUE(reason(cutoff.records[1],"deadline"));
  auto selectCutoff=select({cutoff},Mode::GEOMETRIC_DIVERSE,limits,[]{return false;});
  EXPECT_EQ(1u,selectCutoff.routes.size()); EXPECT_TRUE(selectCutoff.cutoff);
  auto invalid=prepare(graph,5,raw,0,0,motion,limits,[](Id,Id){return false;});
  EXPECT_FALSE(invalid.referenceReady); EXPECT_TRUE(reason(invalid.records[0],"raw_collision"));
  EXPECT_TRUE(select({invalid},Mode::GENERIC_DISTANCE,limits).routes.empty());
  Graph zero({{0,0,0},{0,0,0},{0,0,0}},{{0,2},{0,1},{1,2}});
  std::vector<Path> zeros={{{0,2},0},{{0,1,2},0}};
  auto p=prepare(zero,2,zeros,0,0,motion,limits,adjacency(zero));
  EXPECT_TRUE(reason(p.records[1],"zero_length_reference"));
  motion.speed=0; EXPECT_THROW(prepare(graph,5,raw,0,0,motion,limits,adjacency(graph)),std::invalid_argument);
}
TEST(RouteCandidates, EqualCostTieUsesStableRouteSequence) {
  Graph graph({{0,0,0},{1,0,0},{1,1,0},{2,0,0},{1,-1,0}},
      {{0,1},{1,3},{0,2},{2,3},{0,4},{4,3}});
  std::vector<Path> raw={{{0,1,3},0},{{0,4,3},0},{{0,2,3},0}};
  Limits limits; limits.lengthRatio=2; limits.timeRatio=2; limits.extraYaw=10; limits.routesPerGoal=2;
  MotionConfig motion; motion.yawPenalty=0;
  auto p=prepare(graph,3,raw,0,0,motion,limits,adjacency(graph));
  for(auto mode:{Mode::GENERIC_DISTANCE,Mode::GEOMETRIC_DIVERSE}) {
    auto s=select({p},mode,limits); ASSERT_EQ(2u,s.routes.size());
    EXPECT_EQ((Sequence{0,2,3}),p.records[s.routes[1].poolIndex].raw);
  }
}
TEST(RouteCandidates, MidCandidateCutoffNeverCommitsPartialRoute) {
  auto graph=fixture(); auto raw=routes(graph); Limits limits; MotionConfig motion;
  int ticks=0;
  auto p=prepare(graph,5,raw,0,0,motion,limits,adjacency(graph),[&]{return ++ticks<3;});
  EXPECT_TRUE(p.referenceReady); EXPECT_TRUE(p.cutoff); EXPECT_EQ(1u,p.feasible.size());
  EXPECT_TRUE(reason(p.records[1],"deadline"));
}
TEST(RouteCandidates, ShortcutOrderMatchesGreedySourceAndTieBreaking) {
  EXPECT_EQ((Sequence{0,2,3}),shortcut({0,1,2,3},[](Id a,Id b){return !(a==0 && b==3);}));
  EXPECT_EQ((Sequence{0,1,2,3}),shortcut({0,1,2,3},[](Id,Id){return false;}));
  EXPECT_EQ((Sequence{0,3}),shortcut({0,1,2,3},[](Id,Id){return true;}));
  EXPECT_TRUE(shortcut({},[](Id,Id){return true;}).empty());
}
int main(int argc,char** argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
