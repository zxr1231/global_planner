#include <gtest/gtest.h>
#include "singleAnchorFixture.h"
using namespace singleAnchorFixture;
Input scene(){
 Input in;in.map.version=in.expectedVersion=7;in.map.resolution=.25;in.map.mapMin={-6,-6,0};in.map.mapMax={6,6,3};in.map.dimensions={48,48,12};in.map.pMinLog=-1;in.map.pOccLog=.5;in.map.occupancy.assign(48*48*12,-1);in.map.inflated.assign(48*48*12,false);
 in.positions={{-4,0,1},{4,0,1},{0,.1,1},{0,.2,1},{0,.3,1},{0,.4,1},{0,.5,1},{0,.6,1},{0,.7,1},{0,.8,1},{0,.9,1},{0,1.,1}};in.edges={{0,1}};in.raw=in.reference={0,1};in.goal=1;
 in.legacy.horizontalFov=in.legacy.verticalFov=1.57;in.legacy.dmax=2;in.legacy.planningMin=in.map.mapMin;in.legacy.planningMax=in.map.mapMax;return in;
}
size_t address(const Input& in,Eigen::Vector3d p){Eigen::Vector3i q=((p-in.map.mapMin)/in.map.resolution).array().floor().cast<int>();return (q.x()*48+q.y())*12+q.z();}
TEST(SingleAnchor, NegativeSkipsEvenMalformedContext){Input in;in.accepted=false;auto s=generateAnchors(in);
 EXPECT_TRUE(s.skipped);
 EXPECT_TRUE(s.records.empty());}
TEST(SingleAnchor, DeterministicStratifiedPoolAndUnchangedReference){auto in=scene();auto original=in.reference;auto a=generateAnchors(in),b=generateAnchors(in);ASSERT_EQ(10u,a.ordered.size());ASSERT_EQ(5u,a.pool.size());
 EXPECT_EQ(a.pool,b.pool);for(size_t j=0;j<5;++j)EXPECT_EQ(a.ordered[j*9/4],a.pool[j]);
 EXPECT_EQ(original,in.reference);for(auto i:a.pool){const auto& r=a.records[i];
 EXPECT_EQ((Sequence{0,r.anchor,1}),r.path);
 EXPECT_TRUE(r.reasons.empty());
 EXPECT_LE(r.motion.time,10.);}}
TEST(SingleAnchor, EnumerationCapIncludesRejectedPairs){auto in=scene();auto a=generateAnchors(in,2);
 EXPECT_TRUE(a.truncated);
 EXPECT_EQ(2u,a.records.size());
 EXPECT_TRUE(a.pool.empty());auto b=generateAnchors(in,12);
 EXPECT_FALSE(b.truncated);
 EXPECT_EQ(12u,b.records.size());}
TEST(SingleAnchor, CollinearAndZeroSegmentsNotCandidates){auto in=scene();in.positions={{-4,0,1},{4,0,1},{0,0,1},{-4,0,1}};auto s=generateAnchors(in);
 EXPECT_TRUE(s.pool.empty());
 EXPECT_NE(s.records[2].reasons.end(),std::find(s.records[2].reasons.begin(),s.records[2].reasons.end(),"no_geometric_change"));
 EXPECT_NE(s.records[3].reasons.end(),std::find(s.records[3].reasons.begin(),s.records[3].reasons.end(),"zero_inserted_segment"));}
TEST(SingleAnchor, UnknownAndInflationRejectAnchorsAndSegments){
 auto in=scene();in.positions.resize(3);in.positions[2]={0,1,1};
 auto i=address(in,{0,1,1});in.map.occupancy[i]=-2;
 EXPECT_TRUE(generateAnchors(in).pool.empty());
 in.map.occupancy[i]=-1;in.map.inflated[i]=true;
 EXPECT_TRUE(generateAnchors(in).pool.empty());
 in.map.inflated[i]=false;
 ASSERT_EQ(1u,generateAnchors(in).pool.size());
 in.map.inflated[address(in,{-2,.5,1})]=true;
 EXPECT_TRUE(generateAnchors(in).pool.empty());
}
TEST(SingleAnchor, DuplicatePositionsDoNotFillPool){auto in=scene();in.positions.resize(3);in.positions.push_back(in.positions[2]);auto s=generateAnchors(in);ASSERT_EQ(1u,s.pool.size());
 EXPECT_EQ(2u,s.records[s.pool[0]].anchor);
 EXPECT_EQ("position_sequence_duplicate",s.records[3].reasons.back());}
TEST(SingleAnchor, VersionUnsafeReferenceAndDegenerateFailClosed){auto in=scene();in.expectedVersion=8;
 EXPECT_THROW(generateAnchors(in),ContextError);
 in=scene();in.map.inflated[address(in,{-4,0,1})]=true;
 EXPECT_THROW(generateAnchors(in),ContextError);
 in=scene();in.positions[1]=in.positions[0];
 EXPECT_THROW(generateAnchors(in),ContextError);}
TEST(SingleAnchor, MotionThresholdsInclusiveAndYawWrap){Motion ref{8,0,8},m{10,std::acos(-1.)/2,10};std::vector<std::string> errors;motionRejections(m,ref,Limits{},"final",errors);
 EXPECT_TRUE(errors.empty());m.time=std::nextafter(10.,11.);motionRejections(m,ref,Limits{},"final",errors);
 EXPECT_EQ((std::vector<std::string>{"final_time"}),errors);
 EXPECT_NEAR(.02,angleDistance(std::acos(-1.)-.01,-std::acos(-1.)+.01),1e-12);auto in=scene();Graph graph(in.positions,{});auto motion=measure(graph,{0,2,1},0,0,in.motion);
 EXPECT_NEAR(motion.time,motion.length/in.motion.speed+in.motion.yawPenalty*motion.yaw/in.motion.angularSpeed,1e-12);}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
