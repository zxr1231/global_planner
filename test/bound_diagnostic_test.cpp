#include <gtest/gtest.h>
#include "boundDiagnostic.h"
#include "fixtureAllocation.h"
using namespace boundDiagnostic;
mapManager::OccupancyMapSnapshot grid(){
 mapManager::OccupancyMapSnapshot m;m.dimensions={7,6,5};m.mapMin={-2,-1,0};m.resolution=.2;m.pMinLog=-1;
 m.occupancy.assign(210,-2);for(size_t i=0;i<m.occupancy.size();++i)if(i%3==0)m.occupancy[i]=-1;
 return m;
}
int64_t brute(const mapManager::OccupancyMapSnapshot& m,Box roi,const std::vector<Box>& boxes,const std::vector<uint32_t>& ref){
 int64_t n=0;for(uint32_t id=0;id<m.occupancy.size();++id){auto p=index(m,id);
  if(m.occupancy[id]>=m.pMinLog||!inside(roi,p)||std::find(ref.begin(),ref.end(),id)!=ref.end())continue;
  for(auto b:boxes)if(inside(b,p)){++n;break;}
 }return n;
}
TEST(BoundDiagnostic, ExactUnionTwoAndThreeWithOverlapAndClipping){
 auto m=grid();Vec low{{-1.8,-.8,.2}},high{{-.8,-.2,.8}};std::vector<uint32_t> ref{37,38};
 Box a{{{-1,1,1}},{{4,5,3}}},b{{{2,0,2}},{{8,4,4}}},c{{{1,2,0}},{{5,3,2}}},empty;
 for(bool q:{false,true}){Prefix p(m,low,high,ref,q);auto marked=q?ref:std::vector<uint32_t>{};
  for(auto boxes:{std::vector<Box>{a},std::vector<Box>{a,b},std::vector<Box>{a,b,c},std::vector<Box>{a,a},std::vector<Box>{empty,b},std::vector<Box>{empty}})
   EXPECT_EQ(brute(m,p.roi(),boxes,marked),p.unionCount(boxes));
  EXPECT_EQ(brute(m,p.roi(),{p.roi()},marked),p.total());
 }
}
TEST(BoundDiagnostic, ExhaustiveSmallBoxQueries){
 auto m=grid();Prefix p(m,{{-10,-10,-10}},{{10,10,10}},{},false);
 for(int x=-1;x<8;++x)for(int y=-1;y<7;++y)for(int z=-1;z<6;++z){Box b{{{x,y,z}},{{x+2,y+2,z+2}}};EXPECT_EQ(brute(m,p.roi(),{b},{}),p.count(b));}
}
TEST(BoundDiagnostic, RegionUsesClosedCentersAndEmptyIntersections){
 auto m=grid();Vec center;for(int k=0;k<3;++k)center[k]=m.mapMin[k]+2.5*m.resolution;
 Prefix p(m,center,center,{},false);EXPECT_EQ((std::array<int,3>{{2,2,2}}),p.roi().lo);EXPECT_EQ(p.roi().lo,p.roi().hi);
 Prefix empty(m,{{100,100,100}},{{101,101,101}},{},false);EXPECT_EQ(0,empty.total());EXPECT_EQ(0u,empty.storageBytes());
 auto b=envelope(m,center,center,0);EXPECT_TRUE(inside(b,{{2,2,2}}));
 EXPECT_TRUE(envelope(m,{{100,100,100}},{{101,101,101}},0).empty());
}
TEST(BoundDiagnostic, InvalidReferenceTimeAndParametersReject){
 auto m=grid();Vec low{{-10,-10,-10}},high{{10,10,10}};
 EXPECT_THROW(Prefix(m,low,high,{0},true),std::invalid_argument); // Known-free is not Unknown.
 EXPECT_THROW(Prefix(m,low,high,{210},true),std::invalid_argument);
 Prefix p(m,low,high,{},false);Route r;r.boxes={p.roi()};EXPECT_THROW(score(p,r,0),std::invalid_argument);
 EXPECT_THROW(p.unionCount({}),std::invalid_argument);
 EXPECT_THROW(p.unionCount({p.roi(),p.roi(),p.roi(),p.roi()}),std::invalid_argument);
 EXPECT_THROW(envelope(m,low,high,-1),std::invalid_argument);
 low[0]=NAN;
 EXPECT_THROW(envelope(m,low,high,1),std::invalid_argument);
 m.resolution=0;EXPECT_THROW(validateMap(m),std::invalid_argument);
}
TEST(BoundDiagnostic, ConditionedBoundContainsHypotheticalChangedWindowObservation){
 auto m=grid();std::vector<uint32_t> ref{1,2};Vec low{{-10,-10,-10}},high{{10,10,10}};
 Prefix q(m,low,high,ref,true);Route r;r.time=2;r.boxes={envelope(m,{{-1.5,-.5,.5}},{{-1.2,-.4,.5}},.4),envelope(m,{{-1.2,-.4,.5}},{{-1,-.3,.5}},.4)};
 auto s=score(q,r,ref.size());EXPECT_EQ(brute(m,q.roi(),r.boxes,ref),s.count);
 EXPECT_GE(s.upper,(ref.size()+double(brute(m,q.roi(),r.boxes,ref)))/r.time);
}
TEST(BoundDiagnostic, ScratchMeasuredReleasedAndHardCapRejectsBeforeAllocation){
 auto m=grid();fixtureAllocation::Ledger l;
 {fixtureAllocation::Scope scope(l);Prefix p(m,{{-10,-10,-10}},{{10,10,10}},{1,2},true);EXPECT_GE(l.peak,p.storageBytes());}
 EXPECT_EQ(0u,l.current);EXPECT_GT(l.allocations,0u);
 fixtureAllocation::Ledger tiny;tiny.limit=1;{fixtureAllocation::Scope scope(tiny);EXPECT_THROW(Prefix(m,{{-10,-10,-10}},{{10,10,10}},{},false),std::bad_alloc);}
 EXPECT_TRUE(tiny.rejected);EXPECT_EQ(0u,tiny.current);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
