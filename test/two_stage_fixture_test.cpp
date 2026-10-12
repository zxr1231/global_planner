#include <gtest/gtest.h>
#include "twoStageFixture.h"
using namespace twoStageFixture;
Input example(){
 Input in;in.map.version=in.expectedVersion=42;in.map.resolution=.25;in.map.mapMin={-6,-6,0};in.map.mapMax={6,6,3};in.map.dimensions={48,48,12};in.map.pMinLog=-1;in.map.pOccLog=.5;in.map.occupancy.assign(48*48*12,-1);in.map.inflated.assign(48*48*12,false);
 auto address=[&](Eigen::Vector3d p){Eigen::Vector3i q=((p-in.map.mapMin)/.25).array().floor().cast<int>();return q.x()*48*12+q.y()*12+q.z();};
 in.map.occupancy[address({0,0,1})]=.5;in.map.inflated[address({0,0,1})]=true;in.map.occupancy[address({3,0,1})]=-2;
 in.positions={{-2,0,1},{-1,1,1},{1,1,1},{2,0,1},{-1,-1,1},{1,-1,1}};in.edges={{0,1},{1,2},{2,3},{0,4},{4,5},{5,3}};in.raw={0,1,2,3};in.reference={0,2,3};in.goal=3;
 in.legacy.horizontalFov=in.legacy.verticalFov=1.57;in.legacy.dmax=2;in.legacy.planningMin=in.map.mapMin;in.legacy.planningMax=in.map.mapMax;
 in.camera.fx=in.camera.fy=1;in.camera.cols=in.camera.rows=in.camera.pixelSkip=1;in.camera.range=3;in.camera.planningMin=in.map.mapMin;in.camera.planningMax=in.map.mapMax;
 in.camera.bodyToCamera<<0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1;return in;
}
TEST(TwoStage, NegativeAndZeroBudgetCannotSubmit){auto in=example();in.accepted=false;auto r=trial(in,1);EXPECT_FALSE(r.entered);EXPECT_EQ(0,r.cameraCalls);EXPECT_TRUE(r.selected.empty());in.accepted=true;r=trial(in,1,0);EXPECT_FALSE(r.complete);EXPECT_EQ(0,r.cameraCalls);EXPECT_TRUE(r.selected.empty());}
TEST(TwoStage, VersionAndUnsafeReferenceRejected){auto in=example();in.expectedVersion=43;EXPECT_EQ("REPLAY_CONTEXT_INVALID",trial(in,1).status);in=example();in.map.occupancy[(16*48+24)*12+4]=.5;EXPECT_EQ("REPLAY_CONTEXT_INVALID",trial(in,1).status);}
TEST(TwoStage, ActualAlternativeAndIndependentHistories){auto in=example();for(int mode:{1,2}){auto r=trial(in,mode,1000000);ASSERT_TRUE(r.complete)<<r.status;EXPECT_EQ(2,r.cameraCalls);EXPECT_TRUE(r.motionValid);EXPECT_TRUE(r.finalSafe);EXPECT_GT(r.ref.gain.uniqueGain,0u);EXPECT_EQ(r.ref.gain.uniqueGain,r.alternative.gain.uniqueGain);EXPECT_TRUE(verify(in,r));}}
TEST(TwoStage, NoPaddingAndIncompleteAlternativeNeverRanked){auto in=example();auto stopped=trial(in,1,1000000,0);EXPECT_FALSE(stopped.complete);EXPECT_EQ("ALTERNATIVE_BUDGET_FALLBACK",stopped.status);EXPECT_EQ(in.reference,stopped.selected);EXPECT_FALSE(stopped.alternative.gain.valid);EXPECT_EQ(0u,stopped.alternative.gain.rawGain);in.edges.resize(3);auto r=trial(in,1,1000000);EXPECT_EQ("SUPPLY_DEFICIT",r.status);EXPECT_EQ(1,r.cameraCalls);EXPECT_TRUE(r.fallback);EXPECT_EQ(in.reference,r.selected);}
TEST(TwoStage, DifferentMapDoesNotReuseVisibleSets){auto in=example();auto a=trial(in,0,1000000);ASSERT_GT(a.ref.gain.uniqueGain,0u);for(auto& v:in.map.occupancy)if(v<in.map.pMinLog)v=in.map.pMinLog;++in.map.version;in.expectedVersion=in.map.version;auto b=trial(in,0,1000000);EXPECT_TRUE(b.complete);EXPECT_EQ(0u,b.ref.gain.uniqueGain);}
TEST(TwoStage, SamplerIncludesBoundedYawAndTranslation){auto in=example();auto samples=sample(in,in.reference);for(size_t i=1;i<samples.size();++i){EXPECT_LE((samples[i].position-samples[i-1].position).norm(),.25+1e-12);EXPECT_LE(angleDistance(samples[i].yaw,samples[i-1].yaw),.1+1e-12);}EXPECT_EQ(position(in,in.goal),samples.back().position);}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
