#include <global_planner/incrementalGainEvaluator.h>
#include <algorithm>
#include <cstring>
#include <set>
#include <stdexcept>

namespace globalPlanner {
namespace {
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
uint64_t bits(double d){if(!std::isfinite(d))throw std::invalid_argument("nonfinite incremental state");if(d==0)d=0;uint64_t n;std::memcpy(&n,&d,8);return n;}
std::vector<uint64_t> signature(const mapManager::OccupancyMapSnapshot& m,const CameraGainConfig& c){
 std::vector<uint64_t> s;auto add=[&](double x){s.push_back(bits(x));};
 add(m.resolution);add(m.pMinLog);add(m.pOccLog);
 for(int a=0;a<3;++a){add(m.mapMin[a]);add(m.mapMax[a]);s.push_back(m.dimensions[a]);add(c.planningMin[a]);add(c.planningMax[a]);}
 add(c.fx);add(c.fy);add(c.cx);add(c.cy);add(c.range);
 for(int r=0;r<4;++r)for(int k=0;k<4;++k)add(c.bodyToCamera(r,k));
 for(int n:{c.cols,c.rows,c.margin,c.pixelSkip})s.push_back(n);
 return s;
}
bool inside(uint32_t id,const Eigen::Vector3i& dims,const CameraVisibilityTrace& t){
 int z=id%dims.z(),y=(id/dims.z())%dims.y(),x=id/(dims.y()*dims.z());
 return x>=t.regionMin.x()&&x<=t.regionMax.x()&&y>=t.regionMin.y()&&y<=t.regionMax.y()&&z>=t.regionMin.z()&&z<=t.regionMax.z();
}
}
IncrementalGainEvaluator::IncrementalGainEvaluator(IncrementalGainMode mode,size_t limit):mode_(mode),limit_(limit){}
IncrementalGainEvaluator::Key IncrementalGainEvaluator::key(const PathGainWaypoint& p){return {bits(p.position.x()),bits(p.position.y()),bits(p.position.z()),bits(p.yaw)};}
size_t IncrementalGainEvaluator::entryBytes(const Key&,const Entry& e){
 // Conservative charged storage (including allocator/node allowances), separate from active output/work scratch.
 const auto& t=*e.trace;
 return sizeof(Entry)+sizeof(Key)+192+sizeof(t)+t.transparent.capacity()*4+t.blockers.capacity()*4+
        t.visible.bucket_count()*sizeof(void*)+t.visible.size()*64+e.blocks.capacity()*4+
        e.blocks.size()*(sizeof(Key)+96);
}
size_t IncrementalGainEvaluator::pathBytes(const Path& p){return sizeof(Path)+128+p.keys.capacity()*sizeof(Key)+p.counts.bucket_count()*sizeof(void*)+p.counts.size()*64;}
uint32_t IncrementalGainEvaluator::block(uint32_t id,const Eigen::Vector3i& d) const{
 const uint32_t z=id%d.z(),y=(id/d.z())%d.y(),x=id/(uint64_t(d.y())*d.z());
 return (x/8*((d.y()+7)/8)+y/8)*((d.z()+7)/8)+z/8;
}
void IncrementalGainEvaluator::unindex(const Key& k,const Entry& e){
 for(auto b:e.blocks){auto i=inverse_.find(b);if(i==inverse_.end())continue;auto& v=i->second;v.erase(std::remove(v.begin(),v.end(),k),v.end());if(v.empty())inverse_.erase(i);}
}
void IncrementalGainEvaluator::clearPaths(){for(const auto& p:paths_)bytes_-=pathBytes(p.second);paths_.clear();}
void IncrementalGainEvaluator::eraseEntry(std::map<Key,Entry>::iterator i,IncrementalGainStats& st){
 clearPaths();bytes_-=entryBytes(i->first,i->second);unindex(i->first,i->second);entries_.erase(i);++st.evictions;
}
bool IncrementalGainEvaluator::makeRoom(size_t n,IncrementalGainStats& st){
 if(n>limit_)return false;
 while(bytes_>limit_-n){if(entries_.empty()){clearPaths();break;}auto victim=entries_.begin();for(auto i=entries_.begin();i!=entries_.end();++i)if(i->second.touched<victim->second.touched)victim=i;eraseEntry(victim,st);}
 return bytes_<=limit_-n;
}
void IncrementalGainEvaluator::clear(){entries_.clear();paths_.clear();inverse_.clear();inverse_.rehash(0);std::vector<int8_t>().swap(previous_);std::vector<uint64_t>().swap(signature_);initialized_=false;bytes_=0;}
IncrementalGainResult IncrementalGainEvaluator::evaluate(const mapManager::OccupancyMapSnapshot& m,const CameraGainConfig& c,const std::vector<IncrementalGainRoute>& routes,bool reset){
 const auto begin=Clock::now();IncrementalGainResult out;auto& st=out.stats;
 try {
  auto sig=signature(m,c);std::set<uint64_t> routeIds;
  for(const auto& r:routes){if(!routeIds.insert(r.id).second||r.samples.empty()||!std::isfinite(r.estimatedTime)||r.estimatedTime<0)throw std::invalid_argument("invalid incremental route");for(const auto& p:r.samples)key(p);st.requestedSamples+=r.samples.size();}
  CameraGainEvaluator camera(m,c);st.preparationMs=ms(begin);
  if(mode_==IncrementalGainMode::FULL){
   auto scoring=Clock::now();std::set<Key> distinct;
   for(const auto& r:routes){for(const auto& p:r.samples)distinct.insert(key(p));auto g=camera.evaluateSamples(r.samples,r.estimatedTime,m.version);if(!g.gain.valid)throw std::runtime_error(g.failureReason);out.routes.push_back(std::move(g.gain));}
   st.distinctPoses=distinct.size();st.recomputedPoses=st.requestedSamples;st.pathMs=ms(scoring);st.reset=true;
   double best=-1;for(size_t i=0;i<out.routes.size();++i)if(out.routes[i].uniqueUtility>best+1e-9){best=out.routes[i].uniqueUtility;out.winner=int(i);}
   st.totalMs=ms(begin);return out;
  }
  const bool persistent=mode_==IncrementalGainMode::REGION||mode_==IncrementalGainMode::DEPENDENCY;
  reset=reset||!initialized_||sig!=signature_||m.version<version_||!persistent;
  if(reset)clear();
  st.reset=reset;
  std::vector<Change> changes;auto diff=Clock::now();
  const auto& state=camera.opticalState();
  if(initialized_){for(size_t i=0;i<state.size();++i)if(state[i]!=previous_[i])changes.push_back({uint32_t(i),previous_[i],state[i]});}
  // A changed payload reusing a version is treated as a reset, never a stale-cache hit.
  if(initialized_&&m.version==version_&&!changes.empty()){clear();st.reset=true;changes.clear();}
  st.changedVoxels=changes.size();for(const auto& ch:changes)st.blockingChanges+=(ch.oldState==1)!=(ch.newState==1);
  st.differenceMs=ms(diff);auto update=Clock::now();
  std::map<Key,std::vector<uint32_t>> affected;
  if(mode_==IncrementalGainMode::DEPENDENCY){
   // Group changes by existing spatial dependency before testing exact D/B membership.
   for(const auto& ch:changes){auto b=inverse_.find(block(ch.id,m.dimensions));if(b!=inverse_.end())for(const auto& k:b->second)affected[k].push_back(ch.id);}
  }
  std::set<Key> dirty;std::map<Key,Delta> deltas;
  for(auto& kv:entries_){
   auto& e=kv.second;auto& t=*e.trace;bool invalid=false;
   if(mode_==IncrementalGainMode::REGION){for(const auto& ch:changes)if(inside(ch.id,m.dimensions,t)){invalid=true;break;}}
   else if(mode_==IncrementalGainMode::DEPENDENCY){
    auto ai=affected.find(kv.first);
    if(ai!=affected.end()){
     // Check blockers first across the whole batch; never apply changes sequentially to stale geometry.
     for(auto id:ai->second)if((previous_[id]==1)!=(state[id]==1)){
      if(std::binary_search(t.transparent.begin(),t.transparent.end(),id)||std::binary_search(t.blockers.begin(),t.blockers.end(),id)){invalid=true;break;}
     }
     if(!invalid){auto& d=deltas[kv.first];size_t old=entryBytes(kv.first,e);
      for(auto id:ai->second)if(std::binary_search(t.transparent.begin(),t.transparent.end(),id)){
       if(state[id]==-1){if(t.visible.insert(id).second){d.added.push_back(id);++st.membershipUpdates;}}
       else if(t.visible.erase(id)){d.removed.push_back(id);++st.membershipUpdates;}
      }
      bytes_=bytes_-old+entryBytes(kv.first,e);
     }
    }
   }
   if(invalid)dirty.insert(kv.first);
  }
  // Recompute only dirty entries that are requested now; drop unused stale entries.
  std::map<Key,PathGainWaypoint> requested;for(const auto& r:routes)for(const auto& p:r.samples)requested.emplace(key(p),p);
  st.distinctPoses=requested.size();
  for(auto i=entries_.begin();i!=entries_.end();){if(dirty.count(i->first)&&!requested.count(i->first)){auto j=i++;eraseEntry(j,st);}else ++i;}
  std::map<Key,std::shared_ptr<CameraVisibilityTrace>> frame;
  std::vector<Key> pending;
  for(const auto& p:requested){auto i=entries_.find(p.first);if(i!=entries_.end()&&!dirty.count(p.first)){i->second.touched=++tick_;frame[p.first]=i->second.trace;++st.reusedPoses;}else pending.push_back(p.first);}
  // Batch scratch is reused by the exact optical kernel. All modes use the same physical rays.
  for(size_t start=0;start<pending.size();start+=16){
   std::vector<PathGainWaypoint> poses;for(size_t i=start;i<std::min(start+16,pending.size());++i)poses.push_back(requested.at(pending[i]));
   auto traced=camera.traceSamples(poses,mode_==IncrementalGainMode::DEPENDENCY);
   for(size_t j=0;j<traced.size();++j){const auto& k=pending[start+j];auto trace=std::make_shared<CameraVisibilityTrace>(std::move(traced[j]));++st.recomputedPoses;
    auto old=entries_.find(k);if(old!=entries_.end()){
     auto& d=deltas[k];for(auto id:old->second.trace->visible)if(!trace->visible.count(id))d.removed.push_back(id);
     for(auto id:trace->visible)if(!old->second.trace->visible.count(id))d.added.push_back(id);
     bytes_-=entryBytes(k,old->second);unindex(k,old->second);entries_.erase(old);
    }
    frame[k]=trace;
    if(mode_!=IncrementalGainMode::FULL){Entry e;e.trace=trace;e.touched=++tick_;
     if(mode_==IncrementalGainMode::DEPENDENCY){for(auto id:trace->transparent)e.blocks.push_back(block(id,m.dimensions));for(auto id:trace->blockers)e.blocks.push_back(block(id,m.dimensions));std::sort(e.blocks.begin(),e.blocks.end());e.blocks.erase(std::unique(e.blocks.begin(),e.blocks.end()),e.blocks.end());e.blocks.shrink_to_fit();}
     const auto charge=entryBytes(k,e);if(makeRoom(charge,st)){bytes_+=charge;for(auto b:e.blocks)inverse_[b].push_back(k);entries_.emplace(k,std::move(e));}
    }
   }
  }
  st.updateMs=ms(update);auto pathStart=Clock::now();
  // Expire paths removed by the caller. Geometry changes are checked by exact sample keys below.
  for(auto i=paths_.begin();i!=paths_.end();){if(!routeIds.count(i->first)){bytes_-=pathBytes(i->second);i=paths_.erase(i);}else ++i;}
  for(const auto& r:routes){
   Path path;for(const auto& p:r.samples)path.keys.push_back(key(p));
   auto old=paths_.find(r.id);
   bool incremental=mode_==IncrementalGainMode::DEPENDENCY&&old!=paths_.end()&&old->second.keys==path.keys;
   if(incremental){bytes_-=pathBytes(old->second);path=std::move(old->second);paths_.erase(old);
    for(const auto& k:path.keys){auto di=deltas.find(k);if(di==deltas.end())continue;
     for(auto id:di->second.removed){auto i=path.counts.find(id);if(i==path.counts.end()||!i->second)throw std::logic_error("path removal underflow");if(!--i->second)path.counts.erase(i);--path.raw;++st.propagatedMemberships;}
     for(auto id:di->second.added){++path.counts[id];++path.raw;++st.propagatedMemberships;}
    }
   }else{
    if(old!=paths_.end()){bytes_-=pathBytes(old->second);paths_.erase(old);}
    for(const auto& k:path.keys){const auto& v=frame.at(k)->visible;path.raw+=v.size();for(auto id:v)++path.counts[id];}
   }
   PathGainEvaluation g;g.valid=true;g.mapVersion=m.version;g.sampleCount=r.samples.size();g.rawGain=path.raw;g.uniqueGain=path.counts.size();g.estimatedTime=r.estimatedTime;g.uniqueUtility=r.estimatedTime>1e-6?g.uniqueGain/r.estimatedTime:0;g.duplicateRatio=g.rawGain?1.-double(g.uniqueGain)/g.rawGain:0;
   for(const auto& count:path.counts)g.uniqueAddresses.push_back(count.first);
   std::sort(g.uniqueAddresses.begin(),g.uniqueAddresses.end());
   PathGainEvaluator::AddressSet history;for(const auto& k:path.keys){uint64_t marginal=0;for(auto id:frame.at(k)->visible)marginal+=history.insert(id).second;g.marginalGains.push_back(marginal);}
   if(history.size()!=g.uniqueGain)throw std::logic_error("incremental path invariant failed");
   out.routes.push_back(std::move(g));
   bool allResident=true;for(const auto& k:path.keys)if(!entries_.count(k)){allResident=false;break;}
   if(mode_==IncrementalGainMode::DEPENDENCY&&allResident){size_t charge=pathBytes(path);if(charge<=limit_-std::min(bytes_,limit_)){bytes_+=charge;paths_.emplace(r.id,std::move(path));}}
  }
  st.pathMs=ms(pathStart);
  if(persistent){previous_=state;signature_=std::move(sig);version_=m.version;initialized_=true;}
  else clear();
  // Charge retained map-state vector and bookkeeping separately in the report.
  st.cacheBytes=bytes_+previous_.capacity()+signature_.capacity()*sizeof(uint64_t);
  while(st.cacheBytes>limit_&&!entries_.empty()){eraseEntry(entries_.begin(),st);st.cacheBytes=bytes_+previous_.capacity()+signature_.capacity()*sizeof(uint64_t);}
  if(st.cacheBytes>limit_){clear();st.cacheBytes=0;}
  double best=-1;for(size_t i=0;i<out.routes.size();++i)if(out.routes[i].uniqueUtility>best+1e-9){best=out.routes[i].uniqueUtility;out.winner=int(i);}
  st.totalMs=ms(begin);return out;
 }catch(...){clear();throw;}
}
}
