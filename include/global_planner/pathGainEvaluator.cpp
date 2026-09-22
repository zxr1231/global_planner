#include <global_planner/pathGainEvaluator.h>
#include <global_planner/utils.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace globalPlanner{

PathGainEvaluator::PathGainEvaluator(const mapManager::OccupancyMapSnapshot& snapshot,
									 const PathGainVisibilityConfig& config)
	: snapshot_(snapshot), config_(config){
	const int64_t count = static_cast<int64_t>(snapshot_.dimensions(0)) *
		static_cast<int64_t>(snapshot_.dimensions(1)) * snapshot_.dimensions(2);
	if (snapshot_.resolution <= 0.0 || count <= 0 ||
		static_cast<int64_t>(snapshot_.occupancy.size()) != count ||
		static_cast<int64_t>(snapshot_.inflated.size()) != count){
		throw std::invalid_argument("invalid frozen occupancy snapshot");
	}
	if (!std::isfinite(config_.horizontalFov) || !std::isfinite(config_.verticalFov) ||
		!std::isfinite(config_.dmax) || config_.horizontalFov <= 0.0 ||
		config_.verticalFov <= 0.0 || config_.verticalFov >= M_PI || config_.dmax <= 0.0){
		throw std::invalid_argument("invalid planner visibility configuration");
	}
}

bool PathGainEvaluator::inBounds(const Eigen::Vector3i& index) const{
	return (index.array() >= 0).all() && (index.array() < snapshot_.dimensions.array()).all();
}

Eigen::Vector3i PathGainEvaluator::positionToIndex(const Eigen::Vector3d& position) const{
	return ((position-snapshot_.mapMin)/snapshot_.resolution).array().floor().cast<int>();
}

Eigen::Vector3d PathGainEvaluator::indexToPosition(const Eigen::Vector3i& index) const{
	return snapshot_.mapMin + (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
		snapshot_.resolution;
}

PathGainEvaluator::Address PathGainEvaluator::address(const Eigen::Vector3i& index) const{
	const uint64_t value = static_cast<uint64_t>(index(0)) * snapshot_.dimensions(1) *
		snapshot_.dimensions(2) + static_cast<uint64_t>(index(1)) * snapshot_.dimensions(2) +
		static_cast<uint64_t>(index(2));
	if (value > std::numeric_limits<Address>::max()) throw std::overflow_error("voxel address overflow");
	return static_cast<Address>(value);
}

bool PathGainEvaluator::inflatedAt(const Eigen::Vector3d& position) const{
	const Eigen::Vector3i index = this->positionToIndex(position);
	return !this->inBounds(index) || snapshot_.inflated[this->address(index)];
}

bool PathGainEvaluator::lineOccluded(const Eigen::Vector3d& target,
									 const Eigen::Vector3d& viewpoint) const{
	if (this->inflatedAt(target) || this->inflatedAt(viewpoint)) return true;
	const Eigen::Vector3d delta = viewpoint-target;
	const double distance = delta.norm();
	if (distance <= 0.0) return false;
	const int steps = static_cast<int>(distance/snapshot_.resolution);
	const Eigen::Vector3d increment = delta.normalized()*snapshot_.resolution;
	for (int step=1; step<steps; ++step){
		if (this->inflatedAt(target + step*increment)) return true;
	}
	return false;
}

PathGainEvaluator::AddressSet PathGainEvaluator::visibleUnknown(
		const Eigen::Vector3d& viewpoint, double yaw) const{
	AddressSet visible;
	if (!viewpoint.allFinite() || !std::isfinite(yaw) || this->inflatedAt(viewpoint)) return visible;
	const double zRange = config_.dmax*std::tan(config_.verticalFov/2.0);
	const Eigen::Vector3d scanMin = viewpoint-Eigen::Vector3d(config_.dmax, config_.dmax, zRange);
	const Eigen::Vector3d scanMax = viewpoint+Eigen::Vector3d(config_.dmax, config_.dmax, zRange);
	Eigen::Vector3i first, last;
	for (int axis=0; axis<3; ++axis){
		const double lower = std::max(std::max(scanMin(axis), config_.planningMin(axis)),
			snapshot_.mapMin(axis));
		const double upper = std::min(std::min(scanMax(axis), config_.planningMax(axis)),
			snapshot_.mapMax(axis));
		if (lower > upper) return visible;
		first(axis) = std::max(0, static_cast<int>(std::ceil(
			(lower-snapshot_.mapMin(axis))/snapshot_.resolution-0.5)));
		last(axis) = std::min(snapshot_.dimensions(axis)-1, static_cast<int>(std::floor(
			(upper-snapshot_.mapMin(axis))/snapshot_.resolution-0.5)));
		if (first(axis) > last(axis)) return visible;
	}
	const double maxDistanceSquared = config_.dmax*config_.dmax;
	for (int x=first(0); x<=last(0); ++x){
		for (int y=first(1); y<=last(1); ++y){
			for (int z=first(2); z<=last(2); ++z){
				const Eigen::Vector3i index(x,y,z);
				const Address voxelAddress = this->address(index);
				if (snapshot_.occupancy[voxelAddress] >= snapshot_.pMinLog ||
					snapshot_.inflated[voxelAddress]) continue;
				const Eigen::Vector3d target = this->indexToPosition(index);
				const Eigen::Vector3d delta = target-viewpoint;
				if (delta.squaredNorm() > maxDistanceSquared+1e-12) continue;
				if (std::hypot(delta(0), delta(1)) <= 1e-12) continue;
				if (globalPlanner::angleDiff(std::atan2(delta(1),delta(0)), yaw) >
					config_.horizontalFov/2.0+1e-12) continue;
				if (!this->lineOccluded(target, viewpoint)) visible.insert(voxelAddress);
			}
		}
	}
	return visible;
}

std::vector<PathGainSample> PathGainEvaluator::samplePath(
		const std::vector<PathGainWaypoint>& waypoints, double spacing) const{
	if (!std::isfinite(spacing) || spacing <= 0.0) throw std::invalid_argument("invalid sample spacing");
	std::vector<PathGainSample> samples;
	if (waypoints.empty()) return samples;
	double cumulative = 0.0;
	for (size_t segment=0; segment+1<waypoints.size(); ++segment){
		const Eigen::Vector3d delta = waypoints[segment+1].position-waypoints[segment].position;
		const double length = delta.norm();
		if (length <= 1e-12){
			if (samples.empty()) samples.push_back({waypoints[segment].position,
				waypoints[segment].yaw, static_cast<int>(segment), cumulative});
			continue;
		}
		const Eigen::Vector3d direction = delta/length;
		for (double offset=0.0; offset<length-1e-12; offset+=spacing){
			samples.push_back({waypoints[segment].position+offset*direction,
				waypoints[segment].yaw, static_cast<int>(segment), cumulative+offset});
		}
		cumulative += length;
	}
	const PathGainWaypoint& terminal = waypoints.back();
	if (samples.empty() || (samples.back().position-terminal.position).norm()>1e-12 ||
		globalPlanner::angleDiff(samples.back().yaw, terminal.yaw)>1e-12){
		samples.push_back({terminal.position, terminal.yaw,
			std::max(0, static_cast<int>(waypoints.size())-2), cumulative});
	}
	return samples;
}

PathGainEvaluation PathGainEvaluator::evaluate(const std::vector<PathGainWaypoint>& waypoints,
		double spacing, double estimatedTime, uint64_t expectedMapVersion) const{
	PathGainEvaluation result;
	result.mapVersion = snapshot_.version;
	result.estimatedTime = estimatedTime;
	if (expectedMapVersion != snapshot_.version) return result;
	const std::vector<PathGainSample> samples = this->samplePath(waypoints, spacing);
	AddressSet history;
	for (const PathGainSample& sample : samples){
		const AddressSet visible = this->visibleUnknown(sample.position, sample.yaw);
		result.rawGain += visible.size();
		uint64_t marginal = 0;
		for (Address value : visible){
			if (history.insert(value).second) ++marginal;
		}
		result.marginalGains.push_back(marginal);
	}
	result.sampleCount = samples.size();
	result.uniqueGain = history.size();
	result.duplicateRatio = result.rawGain == 0 ? 0.0 :
		1.0-static_cast<double>(result.uniqueGain)/result.rawGain;
	result.uniqueUtility = estimatedTime > 1e-12 ? result.uniqueGain/estimatedTime : 0.0;
	result.valid = result.rawGain >= result.uniqueGain;
	return result;
}

} // namespace globalPlanner
