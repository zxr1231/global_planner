#include <gtest/gtest.h>
#include <global_planner/opportunityRoutes.h>
#include "twoStageFixture.h"
namespace o=globalPlanner::opportunityRoutes;
using namespace twoStageFixture;
Graph graph(){return Graph({{-4,0,1},{4,0,1},{-1,.2,1},{1,.2,1},{-1,-.2,1},{1,-.2,1}},{{0,1}});}
TEST(OpportunityRoutes, HistoryDedupLostAndNetNotRaw){
 o::Record r;r.motion.time=2;o::annotate(r,{1,2,3},{2,3,4,5});EXPECT_EQ(2u,r.newVoxels);EXPECT_EQ(1u,r.lostVoxels);EXPECT_DOUBLE_EQ(2,r.proxyUtility);
 EXPECT_THROW(o::annotate(r,{1,1},{2}),std::invalid_argument);
}
TEST(OpportunityRoutes, BeamUsesObservationAndCreatesSecondAnchor){
 auto g=graph();o::Config c;c.layerQueries=32;c.beamWidth=1;c.output=5;
 auto observe=[](const Sequence& p){std::vector<uint32_t> s{1};if(std::find(p.begin(),p.end(),4)!=p.end())s.insert(s.end(),{2,3,4,5});if(std::find(p.begin(),p.end(),5)!=p.end())s.push_back(6);return s;};
 auto obs=o::generate(g,{0,1},0,0,MotionConfig{},o::Mode::OBSERVATION,[](Id,Id){return true;},observe,[]{return true;},c);
 auto motion=o::generate(g,{0,1},0,0,MotionConfig{},o::Mode::MOTION,[](Id,Id){return true;},observe,[]{return true;},c);
 ASSERT_EQ(1u,obs.beam.size());EXPECT_EQ((Sequence{0,4,1}),obs.beam[0]);EXPECT_NE(motion.beam,obs.beam);
 EXPECT_GT(obs.secondFeasible,0u);EXPECT_GT(obs.coarseQueries,0u);EXPECT_EQ(0u,motion.coarseQueries);
 auto again=o::generate(g,{0,1},0,0,MotionConfig{},o::Mode::OBSERVATION,[](Id,Id){return true;},observe,[]{return true;},c);EXPECT_EQ(again.beam,obs.beam);
 for(const auto& r:obs.routes){EXPECT_LE(r.motion.time,10);EXPECT_LE(r.motion.length,10);EXPECT_LE(r.motion.yaw,std::acos(-1.)/2);}
}
TEST(OpportunityRoutes, SafetyCancellationActionCapAndBadSet){
 auto g=graph();auto observe=[](const Sequence&){return std::vector<uint32_t>{1};};
 auto none=o::generate(g,{0,1},0,0,MotionConfig{},o::Mode::OBSERVATION,[](Id,Id){return false;},observe,[]{return true;});EXPECT_TRUE(none.routes.empty());EXPECT_EQ(0u,none.coarseQueries);
 EXPECT_THROW(o::generate(g,{0,1},0,0,MotionConfig{},o::Mode::MOTION,[](Id,Id){return true;},observe,[]{return false;}),o::Cancelled);
 o::Config c;c.actions=1;auto capped=o::generate(g,{0,1},0,0,MotionConfig{},o::Mode::MOTION,[](Id,Id){return true;},observe,[]{return true;},c);EXPECT_EQ(1u,capped.actions);EXPECT_TRUE(capped.operationCutoff);
 EXPECT_THROW(o::generate(g,{0,1},0,0,MotionConfig{},o::Mode::OBSERVATION,[](Id,Id){return true;},[](const Sequence&){return std::vector<uint32_t>{2,1};},[]{return true;}),std::invalid_argument);
}
TEST(OpportunityRoutes, CanonicalSamplerMatchesFrozenContractIncludingYawWrap){
 Input in;in.positions={{-4,0,1},{4,0,1},{-1,.2,1}};in.initialYaw=std::acos(-1.)-.01;in.terminalYaw=-std::acos(-1.)+.01;
 Graph g(in.positions,{{0,1}});for(auto path:{Sequence{0,1},Sequence{0,2,1}}){auto a=sample(in,path),b=o::samplePath(g,path,in.initialYaw,in.terminalYaw);ASSERT_EQ(a.size(),b.size());for(size_t k=0;k<a.size();++k){EXPECT_EQ(a[k].position,b[k].position);EXPECT_DOUBLE_EQ(a[k].yaw,b[k].yaw);}}
}
TEST(OpportunityRoutes, SparseEndpointRepresentationPreservesVisibilityAndOcclusion){
 mapManager::OccupancyMapSnapshot m;m.version=1;m.resolution=.25;m.mapMin={-6,-6,0};m.mapMax={6,6,3};m.dimensions={48,48,12};m.pMinLog=-1;m.pOccLog=.5;m.occupancy.assign(48*48*12,-2);m.inflated.assign(m.occupancy.size(),false);
 CameraGainConfig c;c.fx=c.fy=4;c.cols=c.rows=8;c.cx=c.cy=4;c.pixelSkip=2;c.range=3;c.planningMin=m.mapMin;c.planningMax=m.mapMax;c.bodyToCamera<<0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1;
 CameraGainEvaluator dense(m,c);CameraGainEvaluator sparse(m,c);CameraSharedBudget budget;
 auto sets=sparse.evaluatePoseSets({PathGainWaypoint{{0,0,1},0}},1,budget,0);ASSERT_TRUE(sets.valid);
 auto a=dense.visibleUnknown({0,0,1},0);EXPECT_EQ(a,sets.poseSets[0]);EXPECT_NE(a,sparse.visibleUnknown({0,0,1},std::acos(-1.)));
 for(int y=0;y<48;++y)for(int z=0;z<12;++z)m.occupancy[(28*48+y)*12+z]=m.pOccLog;
 CameraGainEvaluator blocked(m,c);auto b=blocked.visibleUnknown({0,0,1},0);EXPECT_LT(b.size(),a.size());for(auto id:b)EXPECT_LT(id/(48*12),28u);
 for(auto& v:m.occupancy)v=m.pMinLog;
 CameraGainEvaluator free(m,c);EXPECT_TRUE(free.visibleUnknown({0,0,1},0).empty());
 EXPECT_GT(sparse.allocatedCapacityBytes(),m.occupancy.size());
}
int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
