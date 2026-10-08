#ifndef GLOBAL_PLANNER_ROUTE_POSE_H
#define GLOBAL_PLANNER_ROUTE_POSE_H
#include <Eigen/Eigen>
#include <cstdint>
#include <mutex>
#include <cmath>
namespace globalPlanner {
struct RoutePose {
 Eigen::Vector3d position=Eigen::Vector3d::Zero();
 double yaw=0, stamp=0;
 uint64_t sequence=0;
 bool valid=false;
};
// Only this mailbox is shared with odometry. The planning worker owns its copy.
class RoutePoseMailbox {
 mutable std::mutex mutex_;
 RoutePose latest_;
 public:
 void update(const Eigen::Vector3d& position,double yaw,double stamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.position=position;latest_.yaw=yaw;latest_.stamp=stamp;++latest_.sequence;
  latest_.valid=position.allFinite() && std::isfinite(yaw) && std::isfinite(stamp);
 }
 RoutePose capture() const {
  std::lock_guard<std::mutex> lock(mutex_);return latest_;
 }
};
}
#endif
