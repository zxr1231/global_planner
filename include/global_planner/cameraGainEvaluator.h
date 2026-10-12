#ifndef GLOBAL_PLANNER_CAMERA_GAIN_EVALUATOR_H
#define GLOBAL_PLANNER_CAMERA_GAIN_EVALUATOR_H
#include <global_planner/pathGainEvaluator.h>
#include <chrono>
#include <limits>
namespace globalPlanner {
struct CameraGainConfig {
 static constexpr const char* model="mapper_matched_frozen_shadow_v1";
 double fx=0,fy=0,cx=0,cy=0,range=0;
 int cols=0,rows=0,margin=0,pixelSkip=0;
 Eigen::Matrix4d bodyToCamera=Eigen::Matrix4d::Identity();
 Eigen::Vector3d planningMin=Eigen::Vector3d::Zero(),planningMax=Eigen::Vector3d::Zero();
};
struct CameraGainBudget {
 uint64_t maxVoxelVisits=std::numeric_limits<uint64_t>::max();
 size_t maxSamples=std::numeric_limits<size_t>::max();
 double maxMilliseconds=std::numeric_limits<double>::infinity();
 bool profile=false;
};
struct CameraGainResult {
 PathGainEvaluation gain;
 std::string failureReason;
 uint64_t voxelVisits=0;
 double evaluationMs=0;
 double endpointMs=0,traversalMs=0;
 uint64_t representativeCount=0;
};
// Caller owns immutable snapshot lifetime; no live-map reads or visibility-result cache.
class CameraGainEvaluator {
public:
 using AddressSet=PathGainEvaluator::AddressSet;
 CameraGainEvaluator(const mapManager::OccupancyMapSnapshot&,const CameraGainConfig&);
 AddressSet visibleUnknown(const Eigen::Vector3d&,double yaw) const;
 CameraGainResult evaluateSamples(const std::vector<PathGainWaypoint>&,double estimatedTime,
                                 uint64_t expectedVersion,const CameraGainBudget& = {}) const;
 size_t rayCount() const{return rays_.size();}
private:
 const mapManager::OccupancyMapSnapshot& map_;
 CameraGainConfig config_;
 std::vector<Eigen::Vector3d> rays_;
 // Immutable compact optical state: occupied / observable Unknown / transparent.
 std::vector<int8_t> opticalState_;
 struct Work {
  CameraGainBudget budget;
  std::chrono::steady_clock::time_point started;
  uint64_t visits=0;
  double endpointMs=0,traversalMs=0;
  uint64_t representativeCount=0;
  std::vector<uint32_t> endpointMarks;
  uint32_t endpointGeneration=0;
  void check() const;
  void visit();
 };
 bool inBounds(const Eigen::Vector3i&) const;
 uint32_t address(const Eigen::Vector3i&) const;
 AddressSet visible(const Eigen::Vector3d&,double,Work&) const;
};
}
#endif
