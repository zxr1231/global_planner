#pragma once
#include <global_planner/cameraGainEvaluator.h>
#include <array>
#include <map>
#include <memory>
#include <unordered_map>

namespace globalPlanner {
enum class IncrementalGainMode { FULL, SNAPSHOT_SHARED, REGION, DEPENDENCY };
struct IncrementalGainRoute {
 uint64_t id=0;
 double estimatedTime=0;
 std::vector<PathGainWaypoint> samples;
};
struct IncrementalGainStats {
 size_t changedVoxels=0,blockingChanges=0,requestedSamples=0,distinctPoses=0;
 size_t recomputedPoses=0,reusedPoses=0,membershipUpdates=0,propagatedMemberships=0;
 size_t evictions=0,cacheBytes=0;
 bool reset=false;
 double preparationMs=0,differenceMs=0,updateMs=0,pathMs=0,totalMs=0;
};
struct IncrementalGainResult {
 std::vector<PathGainEvaluation> routes;
 IncrementalGainStats stats;
 int winner=-1;
};
// Offline prototype. One instance has one writer; input snapshots are immutable.
// Sparse snapshot differences are complete between supplied states, not per-sensor-frame events.
class IncrementalGainEvaluator {
public:
 explicit IncrementalGainEvaluator(IncrementalGainMode,size_t cacheLimit=256u*1024u*1024u);
 IncrementalGainResult evaluate(const mapManager::OccupancyMapSnapshot&,const CameraGainConfig&,
                               const std::vector<IncrementalGainRoute>&,bool reset=false);
 void clear();
private:
 using Key=std::array<uint64_t,4>;
 using Counts=std::unordered_map<uint32_t,uint32_t>;
 struct Entry {
  std::shared_ptr<CameraVisibilityTrace> trace;
  uint64_t touched=0;
  std::vector<uint32_t> blocks;
 };
 struct Path {
  std::vector<Key> keys;
  Counts counts;
  uint64_t raw=0;
 };
 struct Change {uint32_t id;int8_t oldState,newState;};
 struct Delta {std::vector<uint32_t> added,removed;};
 IncrementalGainMode mode_;
 size_t limit_;
 uint64_t tick_=0,version_=0;
 bool initialized_=false;
 std::vector<int8_t> previous_;
 std::vector<uint64_t> signature_;
 std::map<Key,Entry> entries_;
 std::map<uint64_t,Path> paths_;
 std::unordered_map<uint32_t,std::vector<Key>> inverse_;
 size_t bytes_=0;
 static Key key(const PathGainWaypoint&);
 static size_t entryBytes(const Key&,const Entry&);
 static size_t pathBytes(const Path&);
 uint32_t block(uint32_t,const Eigen::Vector3i&) const;
 void unindex(const Key&,const Entry&);
 void eraseEntry(std::map<Key,Entry>::iterator,IncrementalGainStats&);
 bool makeRoom(size_t,IncrementalGainStats&);
 void clearPaths();
};
}
