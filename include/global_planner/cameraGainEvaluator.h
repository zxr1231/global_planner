#ifndef GLOBAL_PLANNER_CAMERA_GAIN_EVALUATOR_H
#define GLOBAL_PLANNER_CAMERA_GAIN_EVALUATOR_H
#include <global_planner/pathGainEvaluator.h>
#include <chrono>
#include <limits>
#include <functional>
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
// Optional isolated caller contract; legacy evaluateSamples/visibleUnknown are unchanged.
struct CameraSharedBudget {
 std::chrono::steady_clock::time_point deadline=std::chrono::steady_clock::time_point::max();
 uint64_t maxVoxelVisits=std::numeric_limits<uint64_t>::max(),voxelVisits=0;
 size_t maxSamples=std::numeric_limits<size_t>::max(),samples=0;
 bool cancelled=false;
 std::function<void()> externalCheck;
 void check() const;
};
struct CameraPoseSetsResult {
 bool valid=false;
 std::string failureReason;
 uint64_t mapVersion=0;
 std::vector<PathGainEvaluator::AddressSet> poseSets;
 double evaluationMs=0;
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
 size_t allocatedCapacityBytes() const{return sizeof(*this)+rays_.capacity()*sizeof(Eigen::Vector3d)+opticalState_.capacity()*sizeof(int8_t);}
 CameraPoseSetsResult evaluatePoseSets(const std::vector<PathGainWaypoint>&,
                                      uint64_t expectedVersion,CameraSharedBudget&) const;
 // Optional per-call representation cap. Existing signatures/config layouts remain intact.
 CameraPoseSetsResult evaluatePoseSets(const std::vector<PathGainWaypoint>&,
                                      uint64_t expectedVersion,CameraSharedBudget&,size_t endpointMarkBudgetBytes) const;
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
  CameraSharedBudget* shared=nullptr;
  size_t endpointMarkBudgetBytes=32u*1024u*1024u;
  void check() const;
  void visit();
 };
 bool inBounds(const Eigen::Vector3i&) const;
 uint32_t address(const Eigen::Vector3i&) const;
 AddressSet visible(const Eigen::Vector3d&,double,Work&) const;
};
}
#endif
