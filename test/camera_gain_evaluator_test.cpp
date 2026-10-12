#include <gtest/gtest.h>
#include <global_planner/cameraGainEvaluator.h>
#include <numeric>
using namespace globalPlanner;
mapManager::OccupancyMapSnapshot grid() {
 mapManager::OccupancyMapSnapshot m;m.version=42;m.resolution=1;m.mapMin=Eigen::Vector3d::Zero();m.mapMax=Eigen::Vector3d(8,5,3);m.dimensions=Eigen::Vector3i(8,5,3);m.pMinLog=-1;m.pOccLog=.5;m.occupancy.assign(120,-1);m.inflated.assign(120,false);return m;
}
CameraGainConfig config() {
 CameraGainConfig c;c.fx=c.fy=1;c.cols=c.rows=c.pixelSkip=1;c.range=5;
 c.bodyToCamera<<0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1;
 c.planningMax=Eigen::Vector3d(8,5,3);return c;
}
uint32_t id(int x,int y,int z){return x*15+y*3+z;}
TEST(CameraGain, UnknownVisibleKnownOccupiedOccludesInflationIsNotOptical) {
 auto m=grid();m.occupancy[id(2,2,1)]=-2;m.occupancy[id(4,2,1)]=-2;m.occupancy[id(6,2,1)]=-2;m.occupancy[id(5,2,1)]=.5;m.inflated[id(2,2,1)]=true;
 CameraGainEvaluator e(m,config());const auto v=e.visibleUnknown({1.5,2.5,1.5},0);
 EXPECT_EQ((CameraGainEvaluator::AddressSet{id(2,2,1),id(4,2,1)}),v);
}
TEST(CameraGain, YawExtrinsicAndInclusiveRegion) {
 auto m=grid();std::fill(m.occupancy.begin(),m.occupancy.end(),-2);
 auto c=config();c.range=3;c.bodyToCamera(0,3)=1;c.planningMin=Eigen::Vector3d(1.5,3.5,1.5);c.planningMax=Eigen::Vector3d(1.5,4.5,1.5);
 CameraGainEvaluator e(m,c);EXPECT_EQ((CameraGainEvaluator::AddressSet{id(1,3,1),id(1,4,1)}),e.visibleUnknown({1.5,.5,1.5},M_PI/2));
}
TEST(CameraGain, EndpointOutsideRejectedOriginCellOmittedZeroPlaneRange) {
 auto m=grid();std::fill(m.occupancy.begin(),m.occupancy.end(),-2);auto c=config();c.range=1;
 CameraGainEvaluator e(m,c);EXPECT_EQ((CameraGainEvaluator::AddressSet{id(2,2,1)}),e.visibleUnknown({1.5,2.5,1.5},0));
 EXPECT_TRUE(e.visibleUnknown({7.5,2.5,1.5},0).empty());EXPECT_TRUE(e.visibleUnknown({-1,2.5,1.5},0).empty());
}
TEST(CameraGain, PixelLatticeMarginSkipAndRepeatedHistory) {
 auto m=grid();std::fill(m.occupancy.begin(),m.occupancy.end(),-2);auto c=config();c.cols=c.rows=6;c.margin=1;c.pixelSkip=2;c.fx=c.fy=10;c.cx=c.cy=2;c.range=3;
 CameraGainEvaluator e(m,c);EXPECT_EQ(4u,e.rayCount());PathGainWaypoint pose;pose.position={1.5,2.5,1.5};
 auto r=e.evaluateSamples({pose,pose},2,m.version);ASSERT_TRUE(r.gain.valid);
 EXPECT_EQ(2*r.gain.uniqueGain,r.gain.rawGain);EXPECT_EQ(0u,r.gain.marginalGains.back());EXPECT_DOUBLE_EQ(.5,r.gain.duplicateRatio);
 auto independent=e.evaluateSamples({pose},2,m.version);EXPECT_EQ(independent.gain.uniqueGain,r.gain.uniqueGain);
}
TEST(CameraGain, InvalidAndBudgetResultsNeverExposePartialGain) {
 auto m=grid();std::fill(m.occupancy.begin(),m.occupancy.end(),-2);CameraGainEvaluator e(m,config());PathGainWaypoint p;p.position={1.5,2.5,1.5};
 EXPECT_FALSE(e.evaluateSamples({p},1,41).gain.valid);EXPECT_FALSE(e.evaluateSamples({},1,42).gain.valid);
 for(int kind=0;kind<3;++kind){CameraGainBudget b;if(kind==0)b.maxSamples=1;if(kind==1)b.maxVoxelVisits=6;if(kind==2)b.maxMilliseconds=0;
  auto r=e.evaluateSamples({p,p},1,42,b);EXPECT_FALSE(r.gain.valid);EXPECT_EQ(0u,r.gain.rawGain);EXPECT_TRUE(r.gain.uniqueAddresses.empty());EXPECT_TRUE(r.gain.marginalGains.empty());}
 p.yaw=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(e.evaluateSamples({p},1,42).gain.valid);
 auto c=config();c.fx=0;EXPECT_THROW(CameraGainEvaluator(m,c),std::invalid_argument);
 m.occupancy.pop_back();
 EXPECT_THROW(CameraGainEvaluator(m,config()),std::invalid_argument);
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
