#ifndef GLOBAL_PLANNER_PATH_GAIN_EVALUATOR_H
#define GLOBAL_PLANNER_PATH_GAIN_EVALUATOR_H

#include <map_manager/occupancyMap.h>
#include <Eigen/Eigen>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace globalPlanner{

struct PathGainVisibilityConfig{
	double horizontalFov = 0.0;
	double verticalFov = 0.0;
	double dmax = 0.0;
	Eigen::Vector3d planningMin = Eigen::Vector3d::Zero();
	Eigen::Vector3d planningMax = Eigen::Vector3d::Zero();
};

struct PathGainWaypoint{
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	double yaw = 0.0;
};

struct PathGainSample{
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	double yaw = 0.0;
	int segment = 0;
	double distanceAlongPath = 0.0;
};

struct PathGainEvaluation{
	bool valid = false;
	uint64_t mapVersion = 0;
	int sampleCount = 0;
	uint64_t rawGain = 0;
	uint64_t uniqueGain = 0;
	std::vector<uint64_t> marginalGains;
	std::vector<uint32_t> uniqueAddresses;
	double duplicateRatio = 0.0;
	double estimatedTime = 0.0;
	double uniqueUtility = 0.0;
};

class PathGainEvaluator{
public:
	using Address = uint32_t;
	using AddressSet = std::unordered_set<Address>;

	PathGainEvaluator(const mapManager::OccupancyMapSnapshot& snapshot,
					  const PathGainVisibilityConfig& config);

	AddressSet visibleUnknown(const Eigen::Vector3d& viewpoint, double yaw) const;
	std::vector<PathGainSample> samplePath(const std::vector<PathGainWaypoint>& waypoints,
									  double spacing) const;
	PathGainEvaluation evaluate(const std::vector<PathGainWaypoint>& waypoints,
							double spacing, double estimatedTime,
							uint64_t expectedMapVersion) const;

private:
	const mapManager::OccupancyMapSnapshot& snapshot_;
	PathGainVisibilityConfig config_;

	bool inBounds(const Eigen::Vector3i& index) const;
	Eigen::Vector3i positionToIndex(const Eigen::Vector3d& position) const;
	Eigen::Vector3d indexToPosition(const Eigen::Vector3i& index) const;
	Address address(const Eigen::Vector3i& index) const;
	bool inflatedAt(const Eigen::Vector3d& position) const;
	bool lineOccluded(const Eigen::Vector3d& target, const Eigen::Vector3d& viewpoint) const;
};

} // namespace globalPlanner

#endif
