#ifndef GLOBAL_PLANNER_ROUTE_SNAPSHOT_H
#define GLOBAL_PLANNER_ROUTE_SNAPSHOT_H
#include <global_planner/pathGainEvaluator.h>
#include <global_planner/PRMKDTree.h>
#include <global_planner/utils.h>
#include <stdexcept>
namespace globalPlanner {
// Read-only view; caller owns one immutable snapshot for graph, collision and scoring.
class RouteSnapshot {
 const mapManager::OccupancyMapSnapshot& map_;
 PathGainVisibilityConfig config_;
 std::vector<double> yaws_;
 int64_t index(const Eigen::Vector3d& p) const {
  if(!p.allFinite()) return -1;
  const auto q=((p-map_.mapMin)/map_.resolution).array().floor().eval();
  if((q<0).any() || (q>=map_.dimensions.cast<double>().array()).any()) return -1;
  const Eigen::Vector3i i=q.cast<int>();
  return int64_t(i(0))*map_.dimensions(1)*map_.dimensions(2)+int64_t(i(1))*map_.dimensions(2)+i(2);
 }
 public:
 RouteSnapshot(const mapManager::OccupancyMapSnapshot& map,const PathGainVisibilityConfig& config,
               const std::vector<double>& yaws):map_(map),config_(config),yaws_(yaws) {
  PathGainEvaluator validation(map,config);
  if(yaws.size()<2) throw std::invalid_argument("at least two legacy yaws required");
 }
 bool unknown(const Eigen::Vector3d& p) const {const auto a=index(p);return a<0 || map_.occupancy[a]<map_.pMinLog;}
 bool inflated(const Eigen::Vector3d& p) const {const auto a=index(p);return a<0 || map_.inflated[a];}
 bool free(const Eigen::Vector3d& p) const {
  const auto a=index(p);return a>=0 && !map_.inflated[a] && map_.occupancy[a]>=map_.pMinLog && map_.occupancy[a]<map_.pOccLog;
 }
 bool freeLine(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const {
  if(!free(a)||!free(b)) return false;
  const Eigen::Vector3d delta=b-a; const double length=delta.norm();
  if(length==0) return true;
  const int steps=int(length/map_.resolution);const Eigen::Vector3d inc=delta.normalized()*map_.resolution;
  for(int i=1;i<steps;++i) if(!free(a+i*inc)) return false;
  return true;
 }
 bool occluded(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const {
  if(inflated(a)||inflated(b)) return true;
  const Eigen::Vector3d delta=b-a;const double length=delta.norm();if(length==0)return false;
  const int steps=int(length/map_.resolution);const Eigen::Vector3d inc=delta.normalized()*map_.resolution;
  for(int i=1;i<steps;++i) if(inflated(a+i*inc)) return true;
  return false;
 }
 int gains(const Eigen::Vector3d& p,std::unordered_map<double,int>& gains) const {
  for(double yaw:yaws_) gains[yaw]=0;
  const double zr=config_.dmax*std::tan(config_.verticalFov/2);int total=0;
  // Deliberately reproduce calculateUnknown's floating scan, not voxel-centre union.
  for(double z=p(2)-zr;z<=p(2)+zr;z+=map_.resolution)
   for(double y=p(1)-config_.dmax;y<=p(1)+config_.dmax;y+=map_.resolution)
    for(double x=p(0)-config_.dmax;x<=p(0)+config_.dmax;x+=map_.resolution) {
     const Eigen::Vector3d target(x,y,z);
     if((target.array()<config_.planningMin.array()).any() || (target.array()>config_.planningMax.array()).any())continue;
     if(!unknown(target)||inflated(target)||(target-p).norm()>config_.dmax||occluded(target,p))continue;
     ++total; const Eigen::Vector3d delta=target-p;const Eigen::Vector3d face(delta(0),delta(1),0);
     for(double yaw:yaws_)if(angleBetweenVectors(face,Eigen::Vector3d(std::cos(yaw),std::sin(yaw),0))<=config_.horizontalFov/2)++gains[yaw];
    }
  return total;
 }
};
}
#endif
