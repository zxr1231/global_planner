#include <gtest/gtest.h>
#include <global_planner/routeSearch.h>
#include <functional>
#include <random>
using namespace globalPlanner::routeSearch;

static std::vector<Path> enumerate(const Graph& graph, Id start, Id goal) {
  std::vector<Path> paths; Sequence prefix{start}; std::set<Id> visited{start};
  std::function<void(Id)> dfs = [&](Id id) {
    if (id==goal) { paths.push_back({prefix,graph.pathCost(prefix)}); return; }
    for (const auto& arc : graph.neighbors(id)) if (!visited.count(arc.to)) {
      visited.insert(arc.to); prefix.push_back(arc.to); dfs(arc.to);
      prefix.pop_back(); visited.erase(arc.to);
    }
  };
  dfs(start);
  std::sort(paths.begin(),paths.end(),[](const Path& a,const Path& b) {
    if (a.cost!=b.cost) return a.cost<b.cost;
    return a.nodes<b.nodes;
  });
  return paths;
}
static void check(const Graph& graph, Id start, Id goal) {
  const auto expected=enumerate(graph,start,goal);
  Budget budget; const auto actual=yen(graph,start,goal,expected.size()+2,budget);
  ASSERT_EQ(expected.size(),actual.paths.size()); EXPECT_TRUE(actual.exhausted);
  EXPECT_EQ(Stop::NONE,actual.stop);
  std::set<Sequence> all, seen;
  for (const auto& path:expected) all.insert(path.nodes);
  for (Id i=0;i<actual.paths.size();++i) {
    const auto& path=actual.paths[i];
    EXPECT_NEAR(expected[i].cost,path.cost,1e-10);
    EXPECT_EQ(1u,all.count(path.nodes)); EXPECT_TRUE(seen.insert(path.nodes).second);
    EXPECT_EQ(path.nodes.size(),std::set<Id>(path.nodes.begin(),path.nodes.end()).size());
  }
  Budget again; const auto repeat=yen(graph,start,goal,expected.size()+2,again);
  ASSERT_EQ(actual.paths.size(),repeat.paths.size());
  for (Id i=0;i<actual.paths.size();++i) EXPECT_EQ(actual.paths[i].nodes,repeat.paths[i].nodes);
  // Every prefix K also agrees in cost with exhaustive enumeration.
  for (Id k=0;k<=std::min<Id>(6,expected.size()+1);++k) {
    Budget b; const auto prefix=yen(graph,start,goal,k,b);
    ASSERT_EQ(std::min(k,expected.size()),prefix.paths.size());
    for (Id i=0;i<prefix.paths.size();++i) EXPECT_NEAR(expected[i].cost,prefix.paths[i].cost,1e-10);
  }
}
TEST(RouteSearch, DirectedSharedPrefixTiesCyclesAndDeficits) {
  Graph graph({{0,0,0},{1,1,0},{1,-1,0},{2,0,0},{3,0,0},{9,9,0}},
      {{0,1},{0,2},{1,3},{2,3},{3,4},{1,2},{2,1},{0,3},{1,3}});
  check(graph,0,4); check(graph,4,0); check(graph,0,5); check(graph,2,2);
  Budget b; const auto paths=yen(graph,0,4,6,b); EXPECT_EQ(5u,paths.paths.size()); EXPECT_TRUE(paths.exhausted);
}
TEST(RouteSearch, ZeroLengthDirectedEdgesAndStableInputOrdering) {
  std::vector<Graph::Position> p={{0,0,0},{0,0,0},{1,0,0},{1,0,0}};
  std::vector<Edge> edges={{0,1},{1,0},{0,2},{1,2},{2,3},{1,3},{0,3}};
  Graph a(p,edges); std::reverse(edges.begin(),edges.end()); Graph b(p,edges);
  check(a,0,3);
  Budget x,y; auto first=yen(a,0,3,20,x); auto second=yen(b,0,3,20,y);
  ASSERT_EQ(first.paths.size(),second.paths.size());
  for(Id i=0;i<first.paths.size();++i) EXPECT_EQ(first.paths[i].nodes,second.paths[i].nodes);
}
TEST(RouteSearch, DenseGraphAndCoincidentVertexCycles) {
  for (bool coincident : {false, true}) {
    std::vector<Graph::Position> positions;
    for (Id i=0;i<6;++i) positions.push_back({coincident?0.0:double(i%3),coincident?0.0:double(i/3),0});
    std::vector<Edge> edges;
    for(Id i=0;i<6;++i) for(Id j=0;j<6;++j) if(i!=j) edges.push_back({i,j});
    Graph graph(positions,edges); check(graph,0,5);
    EXPECT_EQ(65u,enumerate(graph,0,5).size());
  }
}
TEST(RouteSearch, BudgetAndDeadlineRetainMandatoryReference) {
  Graph graph({{0,0,0},{1,1,0},{1,-1,0},{2,0,0}},{{0,1},{0,2},{1,3},{2,3}});
  Budget zero; zero.maxPops=0; auto stopped=yen(graph,0,3,6,zero);
  ASSERT_EQ(1u,stopped.paths.size()); EXPECT_EQ(Stop::POP_LIMIT,stopped.stop);
  EXPECT_EQ(0u,zero.pops); EXPECT_GT(stopped.referencePops,0u); EXPECT_FALSE(stopped.exhausted);
  Budget expired; expired.hasDeadline=true; expired.deadline=std::chrono::steady_clock::now();
  auto timed=yen(graph,0,3,6,expired); ASSERT_EQ(1u,timed.paths.size()); EXPECT_EQ(Stop::DEADLINE,timed.stop);
  for(Id cap=1;cap<30;++cap) {
    Budget limited; limited.maxPops=cap; auto r=yen(graph,0,3,6,limited);
    EXPECT_LE(limited.pops,cap); ASSERT_FALSE(r.paths.empty());
    EXPECT_NEAR(2*std::sqrt(2.0),r.paths.front().cost,1e-10);
  }
}
TEST(RouteSearch, ExclusionsValidationAndNoGraphMutation) {
  Graph graph({{0,0,0},{1,0,0},{2,0,0}},{{0,1},{1,2},{0,2}});
  const auto before=graph.positions(); Budget b;
  auto r=shortest(graph,0,2,b,{1},{{0,2}}); EXPECT_FALSE(r.found);
  EXPECT_EQ(before,graph.positions()); EXPECT_EQ(2u,graph.neighbors(0).size());
  Budget invalid; EXPECT_THROW(shortest(graph,9,0,invalid),std::out_of_range);
  EXPECT_THROW(Graph({{0,0,0}},{{0,2}}),std::out_of_range);
  EXPECT_THROW(Graph({{std::numeric_limits<double>::infinity(),0,0}},{}),std::invalid_argument);
  EXPECT_THROW(graph.pathCost({2,0}),std::invalid_argument);
}
TEST(RouteSearch, ExhaustiveOracleOnRandomDirectedGraphs) {
  std::mt19937 rng(20261008);
  for (int fixture=0;fixture<300;++fixture) {
    const Id n=2+rng()%6; std::vector<Graph::Position> p;
    for(Id i=0;i<n;++i) p.push_back({double(rng()%9),double(rng()%9),double(rng()%3)});
    std::vector<Edge> edges;
    for(Id i=0;i<n;++i) for(Id j=0;j<n;++j) if(i!=j && rng()%100<38) edges.push_back({i,j});
    SCOPED_TRACE(fixture); Graph graph(p,edges); check(graph,0,n-1);
  }
}

TEST(RouteSearch, SuppliedReferenceRetainsOrderingAndUsesOnlyAlternativeBudget) {
  Graph graph({{0,0,0},{1,1,0},{1,-1,0},{2,0,0}},{{0,1},{0,2},{1,3},{2,3}});
  Budget referenceBudget;auto reference=shortest(graph,0,3,referenceBudget);
  Budget a,b;auto expected=yen(graph,0,3,6,a);auto actual=yen(graph,0,3,6,b,&reference.path);
  EXPECT_EQ(0u,actual.referencePops);EXPECT_EQ(a.pops,b.pops);ASSERT_EQ(expected.paths.size(),actual.paths.size());
  for(Id i=0;i<actual.paths.size();++i)EXPECT_EQ(expected.paths[i].nodes,actual.paths[i].nodes);
  Path invalid{{3,0},0};Budget c;EXPECT_THROW(yen(graph,0,3,6,c,&invalid),std::invalid_argument);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
