#pragma once
#include "twoStageFixture.h"
#include <map>
#include <tuple>
namespace singleAnchorFixture {
using namespace twoStageFixture;
constexpr double geometryEpsilon=1e-6;
struct Record {
 size_t edge=0;Id anchor=0;Sequence path;Motion motion;
 std::vector<std::string> reasons;bool motionMeasured=false;
};
struct Supply {
 Motion referenceMotion;std::vector<Record> records;
 std::vector<size_t> ordered,pool;size_t possiblePairs=0;bool truncated=false,skipped=false;
};
inline bool order(const Record& a,const Record& b){
 return std::tie(a.motion.time,a.motion.length,a.anchor,a.edge)<std::tie(b.motion.time,b.motion.length,b.anchor,b.edge);
}
inline Supply generateAnchors(const Input& in,size_t cap=10000){
 Supply out;if(!in.accepted){out.skipped=true;return out;}
 validate(in.motion,Limits{});auto p=prepareContext(in);
 out.referenceMotion=p->reference.simplifiedMotion;
 out.possiblePairs=(in.reference.size()-1)*in.positions.size();
 std::map<Id,size_t> best;
 for(size_t edge=0;edge+1<in.reference.size();++edge)for(Id a=0;a<in.positions.size();++a){
  if(out.records.size()==cap){out.truncated=out.possiblePairs>cap;goto finish;}
  Record r;r.edge=edge;r.anchor=a;
  r.path=in.reference;r.path.insert(r.path.begin()+edge+1,a);
  const auto left=position(in,in.reference[edge]),right=position(in,in.reference[edge+1]),point=position(in,a);
  bool duplicate=false;for(Id ref:in.reference)if((point-position(in,ref)).norm()<=geometryEpsilon)duplicate=true;
  if(duplicate)r.reasons.push_back("reference_vertex_duplicate");
  if((point-left).norm()<=geometryEpsilon||(point-right).norm()<=geometryEpsilon)r.reasons.push_back("zero_inserted_segment");
  if((point-left).norm()+(right-point).norm()-(right-left).norm()<=geometryEpsilon)r.reasons.push_back("no_geometric_change");
  if(r.reasons.empty()){
   r.motion=measure(*p->graph,r.path,in.initialYaw,in.terminalYaw,in.motion);r.motionMeasured=true;
   motionRejections(r.motion,out.referenceMotion,Limits{},"final",r.reasons);
   // Use exactly the existing snapshot inflation/known-free line semantics.
   if(!p->view->freeLine(left,point))r.reasons.push_back("inserted_incoming_unsafe");
   if(!p->view->freeLine(point,right))r.reasons.push_back("inserted_outgoing_unsafe");
  }
  size_t idx=out.records.size();out.records.push_back(r);
  if(r.reasons.empty()){
   auto b=best.find(a);
   // Per-node tie rule is TIME then EDGE; not LENGTH.
   if(b==best.end())best[a]=idx;
   else{
    auto& old=out.records[b->second];
    if(std::tie(r.motion.time,r.edge)<std::tie(old.motion.time,old.edge)){
     old.reasons.push_back("node_insertion_not_best");best[a]=idx;
    }else out.records[idx].reasons.push_back("node_insertion_not_best");
   }
  }
 }
 finish:
 std::vector<size_t> sorted;for(const auto& x:best)sorted.push_back(x.second);
 std::sort(sorted.begin(),sorted.end(),[&](size_t a,size_t b){return order(out.records[a],out.records[b]);});
 for(size_t idx:sorted){
  bool dup=false;for(size_t old:out.ordered)if(sameGeometry(*p->graph,out.records[idx].path,out.records[old].path))dup=true;
  if(dup)out.records[idx].reasons.push_back("position_sequence_duplicate");else out.ordered.push_back(idx);
 }
 if(out.ordered.size()<=5)out.pool=out.ordered;
 else for(size_t j=0;j<5;++j)out.pool.push_back(out.ordered[j*(out.ordered.size()-1)/4]);
 return out;
}
}
