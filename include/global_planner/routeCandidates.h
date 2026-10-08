#ifndef GLOBAL_PLANNER_ROUTE_CANDIDATES_H
#define GLOBAL_PLANNER_ROUTE_CANDIDATES_H
#include <global_planner/routeSearch.h>
#include <functional>
#include <string>

namespace globalPlanner { namespace routeCandidates {
using namespace routeSearch;
using LineCheck = std::function<bool(Id, Id)>;
using Continue = std::function<bool()>;
struct MotionConfig { double speed=1, angularSpeed=1, yawPenalty=1; };
struct Limits {
  std::size_t poolPerGoal=6, routesPerGoal=3, globalRoutes=30;
  double lengthRatio=1.25, timeRatio=1.25, extraYaw=1.5707963267948966;
};
struct Motion { double length=0, yaw=0, time=0; };
inline double angleDistance(double a, double b) {
  return std::abs(std::remainder(a-b, 2*std::acos(-1.0)));
}
inline void validate(const MotionConfig& c, const Limits& l) {
  if (!std::isfinite(c.speed) || c.speed<=0 || !std::isfinite(c.angularSpeed) || c.angularSpeed<=0 ||
      !std::isfinite(c.yawPenalty) || c.yawPenalty<0 ||
      !std::isfinite(l.lengthRatio) || l.lengthRatio<1 ||
      !std::isfinite(l.timeRatio) || l.timeRatio<1 || !std::isfinite(l.extraYaw) || l.extraYaw<0 ||
      !l.poolPerGoal || !l.routesPerGoal || !l.globalRoutes)
    throw std::invalid_argument("route limits/motion configuration");
}
inline Motion measure(const Graph& graph, const Sequence& path, double initialYaw,
                      double goalYaw, const MotionConfig& config) {
  if (path.empty() || !std::isfinite(initialYaw) || !std::isfinite(goalYaw))
    throw std::invalid_argument("empty path/nonfinite yaw");
  Motion result; double previous=initialYaw;
  for (Id i=1;i<path.size();++i) {
    const auto& a=graph.positions().at(path[i-1]); const auto& b=graph.positions().at(path[i]);
    result.length+=std::hypot(std::hypot(b[0]-a[0],b[1]-a[1]),b[2]-a[2]);
    const double yaw=std::atan2(b[1]-a[1],b[0]-a[0]); // same heading convention as DEP
    result.yaw+=angleDistance(previous,yaw); previous=yaw;
  }
  graph.positions().at(path.back());
  result.yaw+=angleDistance(previous,goalYaw);
  result.time=result.length/config.speed+config.yawPenalty*result.yaw/config.angularSpeed;
  if (!std::isfinite(result.length) || !std::isfinite(result.yaw) || !std::isfinite(result.time))
    throw std::overflow_error("route motion overflow");
  return result;
}
// Matches DEP's greedy geometric shortcut order; feasibility comes from a snapshot adapter.
inline Sequence shortcut(const Sequence& path, const LineCheck& freeLine) {
  if(path.empty()) return {};
  if(path.size()<3) return path;
  Sequence result{path.front()}; Id first=0, next=2;
  while(next<path.size()) {
    if(freeLine(path[first],path[next])) {
      if(next==path.size()-1) { result.push_back(path[next]); break; }
      ++next;
    } else {
      result.push_back(path[next-1]);
      if(next==path.size()-1) { result.push_back(path[next]); break; }
      first=next-1; next=first+2;
    }
  }
  return result;
}
inline bool sameGeometry(const Graph& graph, const Sequence& a, const Sequence& b) {
  if(a.size()!=b.size()) return false;
  for(Id i=0;i<a.size();++i) {
    const auto& p=graph.positions().at(a[i]); const auto& q=graph.positions().at(b[i]);
    if(std::hypot(std::hypot(p[0]-q[0],p[1]-q[1]),p[2]-q[2])>1e-6) return false;
  }
  return true;
}
inline double jaccardDistance(const Sequence& a, const Sequence& b) {
  std::set<Edge> x,y;
  for(Id i=1;i<a.size();++i) x.insert({a[i-1],a[i]});
  for(Id i=1;i<b.size();++i) y.insert({b[i-1],b[i]});
  std::size_t common=0; for(const auto& e:x) common+=y.count(e);
  const auto size=x.size()+y.size()-common;
  return size ? 1.0-double(common)/double(size) : 0.0;
}
struct Candidate {
  Id poolIndex=0;
  Sequence raw, simplified;
  Motion rawMotion, simplifiedMotion;
  std::vector<std::string> reasons;
};
struct PreparationCutoff {};
struct Counts { std::size_t rawGenerated=0, motionFeasible=0, postShortcutUnique=0; };
struct GoalPool {
  Id goal=0;
  Counts counts;
  std::vector<Candidate> records;
  std::vector<Id> feasible; // record indices; first reference always index zero
  bool referenceReady=false, cutoff=false;
};
inline void motionRejections(const Motion& motion, const Motion& reference,
                             const Limits& limits, const std::string& stage,
                             std::vector<std::string>& reasons) {
  if(motion.length>reference.length*limits.lengthRatio) reasons.push_back(stage+"_length");
  if(motion.time>reference.time*limits.timeRatio) reasons.push_back(stage+"_time");
  if(motion.yaw>reference.yaw+limits.extraYaw) reasons.push_back(stage+"_yaw");
}
inline GoalPool prepare(const Graph& graph, Id goal, const std::vector<Path>& rawPool,
                        double initialYaw, double goalYaw, const MotionConfig& config,
                        const Limits& limits, const LineCheck& freeLine,
                        const Continue& proceed = [] { return true; }) {
  validate(config,limits);
  if(goal>=graph.size()) throw std::out_of_range("Goal ID");
  if(rawPool.size()>limits.poolPerGoal) throw std::invalid_argument("pool cap exceeded");
  GoalPool result; result.goal=goal; result.counts.rawGenerated=rawPool.size();
  std::set<Sequence> seen;
  for(Id index=0;index<rawPool.size();++index) {
    Candidate c; c.poolIndex=index; c.raw=rawPool[index].nodes;
    if(index && (result.cutoff || !proceed())) {
      result.cutoff=true; c.reasons.push_back("deadline"); result.records.push_back(c); continue;
    }
    if(c.raw.empty() || c.raw.back()!=goal ||
       (!rawPool.front().nodes.empty() && c.raw.front()!=rawPool.front().nodes.front()) ||
       std::set<Id>(c.raw.begin(),c.raw.end()).size()!=c.raw.size()) {
      c.reasons.push_back("invalid_raw_route"); result.records.push_back(c); continue;
    }
    if(!seen.insert(c.raw).second) { c.reasons.push_back("raw_duplicate"); result.records.push_back(c); continue; }
    // Structural invalidity is an API error; collision failures are auditable rejections.
    graph.pathCost(c.raw);
    try {
    const LineCheck checkedLine = [&](Id a, Id b) {
      if(index && !proceed()) throw PreparationCutoff{};
      return freeLine(a,b);
    };
    bool safe=true;
    for(Id i=1;i<c.raw.size();++i) if(!checkedLine(c.raw[i-1],c.raw[i])) safe=false;
    if(!safe) { c.reasons.push_back("raw_collision"); result.records.push_back(c); continue; }
    c.simplified=shortcut(c.raw,checkedLine);
    for(Id i=1;i<c.simplified.size();++i) if(!checkedLine(c.simplified[i-1],c.simplified[i])) safe=false;
    if(!safe) { c.reasons.push_back("shortcut_collision"); result.records.push_back(c); continue; }
    c.rawMotion=measure(graph,c.raw,initialYaw,goalYaw,config);
    c.simplifiedMotion=measure(graph,c.simplified,initialYaw,goalYaw,config);
    if(index && !proceed()) throw PreparationCutoff{};
    if(index==0) result.referenceReady=true;
    else if(!result.referenceReady) c.reasons.push_back("reference_unavailable");
    else {
      const auto& reference=result.records.front();
      if(reference.rawMotion.length==0 || reference.simplifiedMotion.length==0)
        c.reasons.push_back("zero_length_reference");
      motionRejections(c.rawMotion,reference.rawMotion,limits,"raw",c.reasons);
      motionRejections(c.simplifiedMotion,reference.simplifiedMotion,limits,"shortcut",c.reasons);
    }
    if(c.reasons.empty()) {
      ++result.counts.motionFeasible;
      for(Id accepted:result.feasible)
        if(sameGeometry(graph,c.simplified,result.records[accepted].simplified)) {
          c.reasons.push_back("shortcut_duplicate"); break;
        }
      if(c.reasons.empty()) { result.feasible.push_back(result.records.size()); ++result.counts.postShortcutUnique; }
    }
    result.records.push_back(std::move(c));
    } catch(const PreparationCutoff&) {
      result.cutoff=true; c.reasons.push_back("deadline"); result.records.push_back(std::move(c));
    }
  }
  return result;
}
enum class Mode { GENERIC_DISTANCE, GEOMETRIC_DIVERSE };
struct Chosen { Id goal, poolIndex; };
struct Selection {
  std::vector<Chosen> routes;
  std::vector<std::pair<Id,std::size_t>> selectedPerGoal;
  bool cutoff=false;
};
inline Selection select(const std::vector<GoalPool>& pools, Mode mode, const Limits& limits,
                        const Continue& proceed = [] { return true; }) {
  if(!limits.routesPerGoal || !limits.globalRoutes) throw std::invalid_argument("selection cap");
  std::vector<Id> order; std::set<Id> goals; std::size_t references=0;
  for(Id i=0;i<pools.size();++i) {
    if(!goals.insert(pools[i].goal).second) throw std::invalid_argument("duplicate Goal");
    order.push_back(i); references+=pools[i].referenceReady;
  }
  if(references>limits.globalRoutes) throw std::invalid_argument("global cap would drop mandatory reference");
  std::sort(order.begin(),order.end(),[&](Id a,Id b){return pools[a].goal<pools[b].goal;});
  Selection result; std::vector<std::vector<Id>> picked(pools.size());
  for(Id i:order) if(pools[i].referenceReady) {
    if(pools[i].feasible.empty() || pools[i].feasible.front()!=0) throw std::invalid_argument("reference invariant");
    picked[i].push_back(0); result.routes.push_back({pools[i].goal,pools[i].records[0].poolIndex});
  }
  bool added=true;
  while(added && result.routes.size()<limits.globalRoutes) {
    added=false;
    for(Id i:order) {
      if(!proceed()) { result.cutoff=true; added=false; break; }
      if(!pools[i].referenceReady || picked[i].size()>=limits.routesPerGoal) continue;
      Id best=std::numeric_limits<Id>::max(); double bestD=-1;
      for(Id candidate:pools[i].feasible) {
        if(!proceed()) { result.cutoff=true; break; }
        if(std::find(picked[i].begin(),picked[i].end(),candidate)!=picked[i].end()) continue;
        const auto& c=pools[i].records[candidate]; double diversity=1;
        for(Id chosen:picked[i]) diversity=std::min(diversity,jaccardDistance(c.simplified,pools[i].records[chosen].simplified));
        const bool better=best==std::numeric_limits<Id>::max() ||
          (mode==Mode::GEOMETRIC_DIVERSE && diversity>bestD) ||
          ((mode==Mode::GENERIC_DISTANCE || diversity==bestD) &&
           (c.simplifiedMotion.length<pools[i].records[best].simplifiedMotion.length ||
            (c.simplifiedMotion.length==pools[i].records[best].simplifiedMotion.length && c.raw<pools[i].records[best].raw)));
        if(better) { best=candidate; bestD=diversity; }
      }
      if(result.cutoff) { added=false; break; }
      if(best!=std::numeric_limits<Id>::max()) {
        picked[i].push_back(best); result.routes.push_back({pools[i].goal,pools[i].records[best].poolIndex}); added=true;
      }
      if(result.routes.size()==limits.globalRoutes) break;
    }
    if(result.cutoff) break;
  }
  for(Id i:order) result.selectedPerGoal.push_back({pools[i].goal,picked[i].size()});
  return result;
}
}} // namespace
#endif
