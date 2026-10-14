#pragma once
#include <map_manager/occupancyMap.h>
#include <array>
#include <algorithm>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <tuple>
namespace boundDiagnostic {
using Vec=std::array<double,3>;
struct Box {std::array<int,3> lo{{0,0,0}},hi{{-1,-1,-1}};bool empty()const{return hi[0]<lo[0]||hi[1]<lo[1]||hi[2]<lo[2];}};
inline Box intersect(Box a,const Box& b){for(int k=0;k<3;++k){a.lo[k]=std::max(a.lo[k],b.lo[k]);a.hi[k]=std::min(a.hi[k],b.hi[k]);}return a;}
inline std::array<int,3> index(const mapManager::OccupancyMapSnapshot& m,uint32_t id){uint64_t count=uint64_t(m.dimensions.x())*m.dimensions.y()*m.dimensions.z();if(id>=count)throw std::invalid_argument("invalid voxel address");return {{int(id/(m.dimensions.y()*m.dimensions.z())),int((id/m.dimensions.z())%m.dimensions.y()),int(id%m.dimensions.z())}};}
inline bool inside(const Box& b,std::array<int,3> p){for(int k=0;k<3;++k)if(p[k]<b.lo[k]||p[k]>b.hi[k])return false;return true;}
inline void validateMap(const mapManager::OccupancyMapSnapshot& m){
 if(!std::isfinite(m.resolution)||m.resolution<=0||!std::isfinite(m.pMinLog))throw std::invalid_argument("invalid map parameters");
 uint64_t cells=1;
 for(int k=0;k<3;++k){if(m.dimensions[k]<=0||!std::isfinite(m.mapMin[k]))throw std::invalid_argument("invalid map extent");cells*=uint64_t(m.dimensions[k]);if(cells>UINT32_MAX)throw std::invalid_argument("address range exceeded");}
 if(cells!=m.occupancy.size())throw std::invalid_argument("map occupancy size mismatch");
}
inline Box region(const mapManager::OccupancyMapSnapshot& m,const Vec& low,const Vec& high){Box out;for(int k=0;k<3;++k){if(!std::isfinite(low[k])||!std::isfinite(high[k])||low[k]>high[k])throw std::invalid_argument("invalid region");bool found=false;for(int i=0;i<m.dimensions[k];++i){double center=m.mapMin[k]+(double(i)+.5)*m.resolution;if(center>=low[k]&&center<=high[k]){if(!found)out.lo[k]=i;found=true;out.hi[k]=i;}}if(!found){out.hi[k]=-1;out.lo[k]=0;}}return out;}
inline Box envelope(const mapManager::OccupancyMapSnapshot& m,Vec a,Vec b,double radius){if(!std::isfinite(radius)||radius<0)throw std::invalid_argument("invalid radius");Box out;for(int k=0;k<3;++k){if(!std::isfinite(a[k])||!std::isfinite(b[k]))throw std::invalid_argument("nonfinite segment");double lo=(std::min(a[k],b[k])-radius-m.mapMin[k])/m.resolution-.5;double hi=(std::max(a[k],b[k])+radius-m.mapMin[k])/m.resolution-.5;lo=std::ceil(std::nextafter(lo,-INFINITY));hi=std::floor(std::nextafter(hi,INFINITY));out.lo[k]=int(std::max(0.,std::min(double(m.dimensions[k]),lo)));out.hi[k]=int(std::max(-1.,std::min(double(m.dimensions[k]-1),hi)));}return out;}
class Prefix {
 const mapManager::OccupancyMapSnapshot& map_;
 Box roi_;std::array<size_t,3> shape_{{1,1,1}};std::vector<int64_t> table_;
 size_t offset(size_t x,size_t y,size_t z)const{return (x*shape_[1]+y)*shape_[2]+z;}
 int64_t at(size_t x,size_t y,size_t z)const{return table_[offset(x,y,z)];}
public:
 Prefix(const mapManager::OccupancyMapSnapshot& m,const Vec& low,const Vec& high,const std::vector<uint32_t>& reference,bool conditioned):map_(m),roi_(region(m,low,high)){
  validateMap(m);
  if(roi_.empty())return;
  for(int k=0;k<3;++k)shape_[k]=size_t(roi_.hi[k]-roi_.lo[k]+2);
  size_t cells=(shape_[0]-1)*(shape_[1]-1)*(shape_[2]-1);std::vector<uint8_t> marked;
  if(conditioned){marked.assign(cells,0);for(auto id:reference){auto p=index(m,id);if(!inside(roi_,p)||m.occupancy[id]>=m.pMinLog)throw std::invalid_argument("reference not in observable Unknown region");marked[(size_t(p[0]-roi_.lo[0])*(shape_[1]-1)+p[1]-roi_.lo[1])*(shape_[2]-1)+p[2]-roi_.lo[2]]=1;}}
  table_.assign(shape_[0]*shape_[1]*shape_[2],0);
  for(size_t x=1;x<shape_[0];++x)for(size_t y=1;y<shape_[1];++y)for(size_t z=1;z<shape_[2];++z){
   auto id=(uint64_t(x-1+roi_.lo[0])*m.dimensions.y()+y-1+roi_.lo[1])*m.dimensions.z()+z-1+roi_.lo[2];
   size_t local=((x-1)*(shape_[1]-1)+y-1)*(shape_[2]-1)+z-1;
   int64_t value=m.occupancy[id]<m.pMinLog&&(!conditioned||!marked[local]);
   table_[offset(x,y,z)]=value+at(x-1,y,z)+at(x,y-1,z)+at(x,y,z-1)-at(x-1,y-1,z)-at(x-1,y,z-1)-at(x,y-1,z-1)+at(x-1,y-1,z-1);
  }
 }
 int64_t count(Box box)const{
  box=intersect(box,roi_);if(box.empty()||table_.empty())return 0;
  std::array<size_t,3> a,b;for(int k=0;k<3;++k){a[k]=box.lo[k]-roi_.lo[k];b[k]=box.hi[k]-roi_.lo[k]+1;}
  return at(b[0],b[1],b[2])-at(a[0],b[1],b[2])-at(b[0],a[1],b[2])-at(b[0],b[1],a[2])+at(a[0],a[1],b[2])+at(a[0],b[1],a[2])+at(b[0],a[1],a[2])-at(a[0],a[1],a[2]);
 }
 int64_t unionCount(const std::vector<Box>& boxes)const{
  if(boxes.empty()||boxes.size()>3)throw std::invalid_argument("1..3 boxes required");
  int64_t result=0;
  for(unsigned mask=1;mask<(1u<<boxes.size());++mask){Box b=roi_;unsigned n=0;for(size_t k=0;k<boxes.size();++k)if(mask&(1u<<k)){b=intersect(b,boxes[k]);++n;}result+=(n&1?1:-1)*count(b);}
  if(result<0||result>total())throw std::runtime_error("union count range violation");
  return result;
 }
 int64_t total()const{return count(roi_);}size_t storageBytes()const{return table_.capacity()*sizeof(int64_t);}const Box& roi()const{return roi_;}
};
struct Route {size_t id=0,anchor=0,e=0;double time=0,length=0;std::vector<Box> boxes;};
struct Scored {size_t id=0;int64_t count=0;double upper=0;};
inline auto stableKey(const Route& r){return std::make_tuple(r.time,r.length,r.anchor,r.e,r.id);}
inline Scored score(const Prefix& p,const Route& r,size_t referenceSize){if(!std::isfinite(r.time)||r.time<=1e-6)throw std::invalid_argument("invalid route time");auto k=p.unionCount(r.boxes);return {r.id,k,(double(referenceSize)+double(k))/r.time};}
}
