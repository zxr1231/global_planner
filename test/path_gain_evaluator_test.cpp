#include <gtest/gtest.h>
#include <global_planner/pathGainEvaluator.h>
#include <numeric>

namespace{

mapManager::OccupancyMapSnapshot makeSnapshot(){
	mapManager::OccupancyMapSnapshot snapshot;
	snapshot.version = 9;
	snapshot.resolution = 1.0;
	snapshot.mapMin = Eigen::Vector3d::Zero();
	snapshot.mapMax = Eigen::Vector3d(10.0,7.0,3.0);
	snapshot.dimensions = Eigen::Vector3i(10,7,3);
	snapshot.pMinLog = -1.0;
	snapshot.pOccLog = 0.5;
	snapshot.occupancy.assign(10*7*3, 0.0);
	snapshot.inflated.assign(10*7*3, false);
	return snapshot;
}

uint32_t address(const Eigen::Vector3i& index){
	return index(0)*7*3+index(1)*3+index(2);
}

globalPlanner::PathGainVisibilityConfig config(){
	globalPlanner::PathGainVisibilityConfig result;
	result.horizontalFov = M_PI/2.0;
	result.verticalFov = M_PI/2.0;
	result.dmax = 6.0;
	result.planningMin = Eigen::Vector3d::Zero();
	result.planningMax = Eigen::Vector3d(10.0,7.0,3.0);
	return result;
}

}

TEST(PathGainEvaluator, UsesStableAddressesAndInflatedOcclusion){
	auto snapshot = makeSnapshot();
	snapshot.occupancy[address(Eigen::Vector3i(2,3,1))] = -2.0;
	snapshot.occupancy[address(Eigen::Vector3i(5,3,1))] = -2.0;
	globalPlanner::PathGainEvaluator evaluator(snapshot,config());
	auto visible = evaluator.visibleUnknown(Eigen::Vector3d(1.5,3.5,1.5),0.0);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(2,3,1))),1u);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(5,3,1))),1u);

	snapshot.inflated[address(Eigen::Vector3i(3,3,1))] = true;
	globalPlanner::PathGainEvaluator occluded(snapshot,config());
	visible = occluded.visibleUnknown(Eigen::Vector3d(1.5,3.5,1.5),0.0);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(2,3,1))),1u);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(5,3,1))),0u);
}

TEST(PathGainEvaluator, ComputesRawUniqueMarginalAndRepeatability){
	auto snapshot = makeSnapshot();
	for (int x=3; x<=6; ++x) snapshot.occupancy[address(Eigen::Vector3i(x,3,1))] = -2.0;
	globalPlanner::PathGainEvaluator evaluator(snapshot,config());
	std::vector<globalPlanner::PathGainWaypoint> path = {
		{Eigen::Vector3d(1.5,3.5,1.5),0.0},
		{Eigen::Vector3d(2.0,3.5,1.5),0.0}};
	auto first = evaluator.evaluate(path,0.25,2.0,9);
	auto second = evaluator.evaluate(path,0.25,2.0,9);
	ASSERT_TRUE(first.valid);
	EXPECT_GT(first.rawGain,first.uniqueGain);
	EXPECT_EQ(std::accumulate(first.marginalGains.begin(),first.marginalGains.end(),0ull),
		first.uniqueGain);
	EXPECT_EQ(first.rawGain,second.rawGain);
	EXPECT_EQ(first.uniqueGain,second.uniqueGain);
	EXPECT_DOUBLE_EQ(first.uniqueUtility,static_cast<double>(first.uniqueGain)/2.0);
	EXPECT_GT(first.duplicateRatio,0.0);
}

TEST(PathGainEvaluator, FailsClosedOnExpectedVersionMismatch){
	auto snapshot = makeSnapshot();
	globalPlanner::PathGainEvaluator evaluator(snapshot,config());
	std::vector<globalPlanner::PathGainWaypoint> path = {
		{Eigen::Vector3d(1.5,3.5,1.5),0.0}};
	EXPECT_FALSE(evaluator.evaluate(path,0.25,1.0,8).valid);
	EXPECT_THROW(evaluator.samplePath(path,0.0),std::invalid_argument);
}

TEST(PathGainEvaluator, ClipsPlanningRegionAndHandlesYawWrap){
	auto snapshot = makeSnapshot();
	snapshot.occupancy[address(Eigen::Vector3i(0,3,1))] = -2.0;
	snapshot.occupancy[address(Eigen::Vector3i(2,3,1))] = -2.0;
	auto limited = config();
	limited.planningMin(0)=1.0;
	globalPlanner::PathGainEvaluator evaluator(snapshot,limited);
	auto visible=evaluator.visibleUnknown(Eigen::Vector3d(1.5,3.5,1.5),M_PI-0.01);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(0,3,1))),0u);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(2,3,1))),0u);
	visible=evaluator.visibleUnknown(Eigen::Vector3d(1.5,3.5,1.5),-0.01);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(2,3,1))),1u);
	globalPlanner::PathGainEvaluator fullRegion(snapshot,config());
	visible=fullRegion.visibleUnknown(Eigen::Vector3d(1.5,3.5,1.5),-M_PI+0.01);
	EXPECT_EQ(visible.count(address(Eigen::Vector3i(0,3,1))),1u);
}

TEST(PathGainEvaluator, PreservesTerminalYawAcrossZeroLengthSegment){
	auto snapshot=makeSnapshot();
	globalPlanner::PathGainEvaluator evaluator(snapshot,config());
	std::vector<globalPlanner::PathGainWaypoint> path={
		{Eigen::Vector3d(1.5,3.5,1.5),0.0},
		{Eigen::Vector3d(1.5,3.5,1.5),0.5}};
	const auto samples=evaluator.samplePath(path,0.25);
	ASSERT_EQ(samples.size(),2u);
	EXPECT_DOUBLE_EQ(samples.front().yaw,0.0);
	EXPECT_DOUBLE_EQ(samples.back().yaw,0.5);
}

TEST(PathGainEvaluator, RejectsMalformedSnapshot){
	auto snapshot=makeSnapshot();
	snapshot.inflated.pop_back();
	EXPECT_THROW(globalPlanner::PathGainEvaluator(snapshot,config()),std::invalid_argument);
}

TEST(PathGainEvaluator, RanksUniqueCandidatesAndKeepsFirstTie){
	std::vector<globalPlanner::PathGainEvaluation> values(3);
	for (auto& value:values) value.valid=true;
	values[0].uniqueUtility=2.0;
	values[1].uniqueUtility=5.0;
	values[2].uniqueUtility=5.0;
	const auto ranking=globalPlanner::rankUniqueCandidates(values);
	ASSERT_TRUE(ranking.valid);
	EXPECT_EQ(ranking.candidateIndex,1);
	EXPECT_DOUBLE_EQ(ranking.scoreMargin,0.0);
}

TEST(PathGainEvaluator, RankingFailsClosedWithoutValidCandidate){
	std::vector<globalPlanner::PathGainEvaluation> values(2);
	values[0].valid=false;
	values[1].valid=true;
	values[1].uniqueUtility=-1.0;
	const auto ranking=globalPlanner::rankUniqueCandidates(values);
	EXPECT_FALSE(ranking.valid);
	EXPECT_EQ(ranking.candidateIndex,-1);
	EXPECT_EQ(ranking.failureReason,"no_valid_unique_candidate");
}

TEST(PathGainEvaluator, OnlineSelectionUsesUniqueAndOtherModesKeepLegacy){
	globalPlanner::UniqueGainRanking ranking;
	ranking.valid=true;
	ranking.candidateIndex=2;
	auto decision=globalPlanner::decidePathSelection(
		globalPlanner::PathGainMode::UNIQUE_ONLINE,ranking,3,0);
	EXPECT_EQ(decision.candidateIndex,2);
	EXPECT_EQ(decision.selectionMode,"unique");
	EXPECT_TRUE(decision.fallbackReason.empty());
	decision=globalPlanner::decidePathSelection(
		globalPlanner::PathGainMode::UNIQUE_SHADOW,ranking,3,0);
	EXPECT_EQ(decision.candidateIndex,0);
	EXPECT_EQ(decision.selectionMode,"legacy");
}

TEST(PathGainEvaluator, OnlineSelectionFallsBackForInvalidOrOutOfRangeRanking){
	globalPlanner::UniqueGainRanking invalid;
	invalid.failureReason="no_valid_unique_candidate";
	auto decision=globalPlanner::decidePathSelection(
		globalPlanner::PathGainMode::UNIQUE_ONLINE,invalid,3,1);
	EXPECT_EQ(decision.candidateIndex,1);
	EXPECT_EQ(decision.selectionMode,"legacy");
	EXPECT_EQ(decision.fallbackReason,"no_valid_unique_candidate");
	globalPlanner::UniqueGainRanking outside;
	outside.valid=true;
	outside.candidateIndex=4;
	decision=globalPlanner::decidePathSelection(
		globalPlanner::PathGainMode::UNIQUE_ONLINE,outside,3,1);
	EXPECT_EQ(decision.candidateIndex,1);
	EXPECT_EQ(decision.fallbackReason,"unique_candidate_out_of_range");
}

int main(int argc,char** argv){
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}
