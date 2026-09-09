#include <global_planner/dep.h>

namespace globalPlanner {

bool DEP::reachableGainExhausted(int threshold, int& checkedNodes) {
  checkedNodes = 0;
  if (!odomReceived_ || !map_ || prmNodeVec_.empty() ||
      !map_->isInflatedFree(position_)) return false;
  std::queue<std::shared_ptr<PRM::Node>> open;
  std::unordered_set<std::shared_ptr<PRM::Node>> seen;
  const auto knownFreeConnection = [this](const Eigen::Vector3d& a,
                                           const Eigen::Vector3d& b) {
    return (a-b).norm() <= 1e-6 ? map_->isInflatedFree(a) :
                                  map_->isInflatedFreeLine(a,b);
  };
  for (const auto& node : prmNodeVec_) {
    if ((node->pos - position_).norm() <= maxConnectDist_ &&
        knownFreeConnection(position_, node->pos)) {
      seen.insert(node);
      open.push(node);
    }
  }
  while (!open.empty() && ros::ok()) {
    const auto node = open.front(); open.pop();
    std::unordered_map<double, int> gains;
    calculateUnknown(node, gains); // Fresh gains, including nodes excluded by goal filtering.
    ++checkedNodes;
    for (const auto& gain : gains) if (gain.second > threshold) return false;
    for (const auto& next : node->adjNodes) {
      if (prmNodeVec_.count(next) && !seen.count(next) &&
          knownFreeConnection(node->pos, next->pos)) {
        seen.insert(next);
        open.push(next);
      }
    }
  }
  return ros::ok() && checkedNodes > 0;
}

bool DEP::planReturnPath(const Eigen::Vector3d& home, nav_msgs::Path& path) {
  path.poses.clear();
  if (!odomReceived_ || !map_ || !map_->isInflatedFree(position_) ||
      !map_->isInflatedFree(home)) return false;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  if ((position_-home).norm() <= 1e-6) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = home.x(); pose.pose.position.y = home.y();
    pose.pose.position.z = home.z(); pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    return true;
  }
  const auto knownFreeConnection = [this](const Eigen::Vector3d& a,
                                           const Eigen::Vector3d& b) {
    return (a-b).norm() <= 1e-6 ? map_->isInflatedFree(a) :
                                  map_->isInflatedFreeLine(a,b);
  };
  auto start = std::make_shared<PRM::Node>(position_);
  auto goal = std::make_shared<PRM::Node>(home);
  std::vector<std::shared_ptr<PRM::Node>> route;
  if (knownFreeConnection(position_, home)) {
    route = {start, goal};
  } else {
    // Local Dijkstra labels: never mutate PRM nodes or a live priority queue key.
    struct Entry { double cost; std::shared_ptr<PRM::Node> node; };
    struct Compare { bool operator()(const Entry& a, const Entry& b) const {
      return a.cost > b.cost;
    }};
    std::priority_queue<Entry, std::vector<Entry>, Compare> open;
    std::unordered_map<std::shared_ptr<PRM::Node>, double> distance;
    std::unordered_map<std::shared_ptr<PRM::Node>, std::shared_ptr<PRM::Node>> parent;
    std::unordered_set<std::shared_ptr<PRM::Node>> homeNeighbors;
    for (const auto& node : prmNodeVec_) {
      if ((node->pos - home).norm() <= maxConnectDist_ &&
          knownFreeConnection(node->pos, home)) homeNeighbors.insert(node);
      const double d = (node->pos - position_).norm();
      if (d <= maxConnectDist_ && knownFreeConnection(position_, node->pos)) {
        distance[node] = d; parent[node] = start; open.push({d, node});
      }
    }
    bool found = false;
    while (!open.empty() && ros::ok()) {
      const Entry current = open.top(); open.pop();
      if (current.cost > distance.at(current.node)) continue;
      if (current.node == goal) { found = true; break; }
      std::vector<std::shared_ptr<PRM::Node>> neighbors(
          current.node->adjNodes.begin(), current.node->adjNodes.end());
      if (homeNeighbors.count(current.node)) neighbors.push_back(goal);
      for (const auto& next : neighbors) {
        if (next != goal && !prmNodeVec_.count(next)) continue;
        if (!knownFreeConnection(current.node->pos, next->pos)) continue;
        const double d = current.cost + (current.node->pos - next->pos).norm();
        auto old = distance.find(next);
        if (old == distance.end() || d < old->second) {
          distance[next] = d; parent[next] = current.node; open.push({d, next});
        }
      }
    }
    if (!found) return false;
    for (auto node = goal; node != start; node = parent.at(node)) route.push_back(node);
    route.push_back(start);
    std::reverse(route.begin(), route.end());
  }
  std::vector<std::shared_ptr<PRM::Node>> shortened;
  shortcutPath(route, shortened);
  // Recheck every segment, including start/home connectors, before handing off.
  for (size_t i = 1; i < shortened.size(); ++i)
    if (!knownFreeConnection(shortened[i-1]->pos, shortened[i]->pos)) return false;
  for (size_t i = 0; i < shortened.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = shortened[i]->pos.x();
    pose.pose.position.y = shortened[i]->pos.y();
    pose.pose.position.z = shortened[i]->pos.z();
    const Eigen::Vector3d delta = i+1 < shortened.size() ?
        Eigen::Vector3d(shortened[i+1]->pos - shortened[i]->pos) : Eigen::Vector3d::UnitX();
    pose.pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, atan2(delta.y(), delta.x()));
    path.poses.push_back(pose);
  }
  return path.poses.size() >= 2;
}
} // namespace globalPlanner
