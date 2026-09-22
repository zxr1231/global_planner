#include <gtest/gtest.h>
#include <global_planner/pathGainContract.h>

using globalPlanner::PathGainMode;

TEST(PathGainContract, ParsesEveryFrozenMode){
	EXPECT_EQ(globalPlanner::parsePathGainMode("legacy"), PathGainMode::LEGACY);
	EXPECT_EQ(globalPlanner::parsePathGainMode("unique_shadow"), PathGainMode::UNIQUE_SHADOW);
	EXPECT_EQ(globalPlanner::parsePathGainMode("unique_online"), PathGainMode::UNIQUE_ONLINE);
	EXPECT_THROW(globalPlanner::parsePathGainMode("unique"), std::invalid_argument);
}

TEST(PathGainContract, ExposesSelectionSemantics){
	EXPECT_FALSE(globalPlanner::requestsUniqueEvaluation(PathGainMode::LEGACY));
	EXPECT_TRUE(globalPlanner::requestsUniqueEvaluation(PathGainMode::UNIQUE_SHADOW));
	EXPECT_FALSE(globalPlanner::requestsUniqueSelection(PathGainMode::UNIQUE_SHADOW));
	EXPECT_TRUE(globalPlanner::requestsUniqueSelection(PathGainMode::UNIQUE_ONLINE));
}

TEST(PathGainContract, FailsClosedBeforeEvaluatorImplementation){
	EXPECT_NO_THROW(globalPlanner::validatePathGainContract(1, PathGainMode::LEGACY, 0.25, false));
	EXPECT_THROW(globalPlanner::validatePathGainContract(2, PathGainMode::LEGACY, 0.25, false),
		std::invalid_argument);
	EXPECT_THROW(globalPlanner::validatePathGainContract(1, PathGainMode::LEGACY, 0.0, false),
		std::invalid_argument);
	EXPECT_THROW(globalPlanner::validatePathGainContract(
		1, PathGainMode::UNIQUE_SHADOW, 0.25, false), std::logic_error);
	EXPECT_THROW(globalPlanner::validatePathGainContract(
		1, PathGainMode::UNIQUE_ONLINE, 0.25, false), std::logic_error);
	EXPECT_NO_THROW(globalPlanner::validatePathGainContract(
		1, PathGainMode::UNIQUE_SHADOW, 0.25, true));
}

int main(int argc, char** argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
