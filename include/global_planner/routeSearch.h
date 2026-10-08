#ifndef GLOBAL_PLANNER_ROUTE_SEARCH_H
#define GLOBAL_PLANNER_ROUTE_SEARCH_H

// Isolated I2 controls. No ROS, map access, shared PRM state or online call sites.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace globalPlanner { namespace routeSearch {
using Id = std::size_t;
using Sequence = std::vector<Id>;
using Edge = std::pair<Id, Id>;
struct Arc { Id to; double cost; };
class Graph {
 public:
  using Position = std::array<double, 3>;
  // IDs belong to the caller's exported immutable snapshot. Never symmetrize.
  Graph(std::vector<Position> positions, const std::vector<Edge>& edges)
      : positions_(std::move(positions)), adjacency_(positions_.size()) {
    for (const auto& p : positions_)
      for (double v : p) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite position");
    for (const auto& e : edges) {
      if (e.first >= size() || e.second >= size()) throw std::out_of_range("edge ID");
      if (e.first == e.second) continue;
      const auto& a = positions_[e.first]; const auto& b = positions_[e.second];
      const double d = std::hypot(std::hypot(a[0]-b[0], a[1]-b[1]), a[2]-b[2]);
      if (!std::isfinite(d)) throw std::invalid_argument("nonfinite edge length");
      adjacency_[e.first].push_back({e.second, d});
    }
    for (auto& neighbors : adjacency_) {
      std::sort(neighbors.begin(), neighbors.end(), [](const Arc& a, const Arc& b) { return a.to < b.to; });
      neighbors.erase(std::unique(neighbors.begin(), neighbors.end(),
          [](const Arc& a, const Arc& b) { return a.to == b.to; }), neighbors.end());
    }
  }
  Id size() const { return positions_.size(); }
  const std::vector<Arc>& neighbors(Id i) const { return adjacency_.at(i); }
  const std::vector<Position>& positions() const { return positions_; }
  double pathCost(const Sequence& nodes) const {
    double total = 0;
    for (Id i=1; i<nodes.size(); ++i) {
      const auto& neighbors = adjacency_.at(nodes[i-1]);
      auto it = std::lower_bound(neighbors.begin(), neighbors.end(), nodes[i],
          [](const Arc& a, Id id) { return a.to < id; });
      if (it == neighbors.end() || it->to != nodes[i]) throw std::invalid_argument("missing directed edge");
      total += it->cost;
    }
    return total;
  }
 private:
  std::vector<Position> positions_;
  std::vector<std::vector<Arc>> adjacency_;
};
struct Path { Sequence nodes; double cost = 0; };
enum class Stop { NONE, POP_LIMIT, DEADLINE };
struct Budget {
  std::size_t maxPops = std::numeric_limits<std::size_t>::max();
  std::size_t pops = 0;
  bool hasDeadline = false;
  std::chrono::steady_clock::time_point deadline;
  Stop stop = Stop::NONE;
  bool check() {
    if (stop != Stop::NONE) return false;
    if (hasDeadline && std::chrono::steady_clock::now() >= deadline) {
      stop = Stop::DEADLINE; return false;
    }
    if (pops >= maxPops) { stop = Stop::POP_LIMIT; return false; }
    return true;
  }
  bool pop() { if (!check()) return false; ++pops; return true; }
};
struct SearchResult { bool found = false; Path path; Stop stop = Stop::NONE; };
struct PathGreater {
  bool operator()(const Path& a, const Path& b) const {
    if (a.cost != b.cost) return a.cost > b.cost;
    return a.nodes > b.nodes;
  }
};
// Exact comparisons preserve queue ordering. Ties are deterministic, not an epsilon.
inline SearchResult shortest(const Graph& graph, Id start, Id goal, Budget& budget,
    const std::set<Id>& excludedNodes = {}, const std::set<Edge>& excludedEdges = {}) {
  if (start >= graph.size() || goal >= graph.size()) throw std::out_of_range("terminal ID");
  SearchResult result;
  if (excludedNodes.count(start) || excludedNodes.count(goal)) return result;
  std::vector<double> distance(graph.size(), std::numeric_limits<double>::infinity());
  std::vector<Sequence> best(graph.size());
  std::vector<bool> closed(graph.size(), false);
  std::priority_queue<Path, std::vector<Path>, PathGreater> open;
  distance[start] = 0; best[start] = {start}; open.push({{start}, 0});
  while (!open.empty()) {
    if (!budget.pop()) { result.stop=budget.stop; return result; }
    const Path current = open.top(); open.pop(); const Id id = current.nodes.back();
    if (closed[id] || current.cost != distance[id] || current.nodes != best[id]) continue;
    closed[id] = true;
    if (id == goal) { result.found=true; result.path=current; return result; }
    for (const auto& arc : graph.neighbors(id)) {
      if (!budget.check()) { result.stop=budget.stop; return result; }
      if (closed[arc.to] || excludedNodes.count(arc.to) || excludedEdges.count({id,arc.to})) continue;
      const double cost=current.cost+arc.cost;
      if (!std::isfinite(cost)) throw std::overflow_error("path cost overflow");
      Sequence nodes=current.nodes; nodes.push_back(arc.to);
      if (cost < distance[arc.to] || (cost == distance[arc.to] && nodes < best[arc.to])) {
        distance[arc.to]=cost; best[arc.to]=nodes; open.push({nodes,cost});
      }
    }
  }
  return result;
}
struct Routes {
  std::vector<Path> paths;
  bool exhausted = false; // pool genuinely exhausted, not just K reached
  Stop stop = Stop::NONE;
  std::size_t referencePops = 0;
};
inline Routes yen(const Graph& graph, Id start, Id goal, std::size_t k, Budget& alternatives, const Path* suppliedReference = nullptr) {
  Routes result;
  if (start >= graph.size() || goal >= graph.size()) throw std::out_of_range("terminal ID");
  if (k == 0) return result;
  Budget referenceBudget;
  SearchResult reference;
  if (suppliedReference) {
    if(suppliedReference->nodes.empty() || suppliedReference->nodes.front()!=start ||
       suppliedReference->nodes.back()!=goal ||
       std::set<Id>(suppliedReference->nodes.begin(),suppliedReference->nodes.end()).size()!=suppliedReference->nodes.size())
      throw std::invalid_argument("invalid supplied reference");
    reference.found=true; reference.path=*suppliedReference;
    reference.path.cost=graph.pathCost(reference.path.nodes);
  } else reference=shortest(graph,start,goal,referenceBudget);
  result.referencePops=referenceBudget.pops;
  if (!reference.found) { result.exhausted=true; return result; }
  result.paths.push_back(reference.path);
  std::priority_queue<Path,std::vector<Path>,PathGreater> pool;
  std::set<Sequence> seen; seen.insert(reference.path.nodes);
  while (result.paths.size() < k) {
    const auto previous=result.paths.back().nodes;
    for (Id spurIndex=0; spurIndex+1<previous.size(); ++spurIndex) {
      if (!alternatives.check()) { result.stop=alternatives.stop; return result; }
      Sequence root(previous.begin(),previous.begin()+spurIndex+1);
      std::set<Id> bannedNodes(root.begin(),root.end()-1);
      std::set<Edge> bannedEdges;
      for (const auto& accepted : result.paths) {
        if (accepted.nodes.size()>spurIndex+1 &&
            std::equal(root.begin(),root.end(),accepted.nodes.begin()))
          bannedEdges.insert({accepted.nodes[spurIndex],accepted.nodes[spurIndex+1]});
      }
      const auto spur=shortest(graph,root.back(),goal,alternatives,bannedNodes,bannedEdges);
      if (spur.stop!=Stop::NONE) { result.stop=spur.stop; return result; }
      if (spur.found) {
        Sequence nodes=root; nodes.insert(nodes.end(),spur.path.nodes.begin()+1,spur.path.nodes.end());
        if (seen.insert(nodes).second) pool.push({nodes,graph.pathCost(nodes)});
      }
    }
    if (pool.empty()) { result.exhausted=true; return result; }
    if (!alternatives.pop()) { result.stop=alternatives.stop; return result; }
    result.paths.push_back(pool.top()); pool.pop();
  }
  return result;
}
}} // namespace
#endif
