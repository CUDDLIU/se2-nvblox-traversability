#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

struct Graph {
  int64_t nodes;
  const int64_t *offsets, *target;
  const double* costs;
  const uint8_t* blocked;
};

// Reverse Dijkstra need only settle the query start. All unsettled distances
// are >= that distance, so capping them there remains an admissible, consistent
// heuristic even when subsequent lazy collision checks remove edges. This
// avoids expanding remote rooms/floors unrelated to this query.
extern "C" int se2_goal_distances_graphs(int64_t n,const Graph* graphs,int64_t count,
    const uint8_t* goals,int64_t start,double* distance,int64_t* expanded) {
  if(n<=0 || start<0 || start>=n)return 1;
  using Entry=std::pair<double,int64_t>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  std::fill(distance,distance+n,std::numeric_limits<double>::infinity());
  *expanded=0;
  for(int64_t i=0;i<n;++i)if(goals[i]){distance[i]=0.;queue.emplace(0.,i);}
  while(!queue.empty()) {
    auto [value,a]=queue.top();queue.pop();
    if(value>distance[a])continue;
    ++*expanded;
    if(a==start) {
      for(int64_t i=0;i<n;++i)distance[i]=std::min(distance[i],value);
      return 0;
    }
    for(int64_t g=0;g<count;++g) {
      const auto& graph=graphs[g];if(a>=graph.nodes)continue;
      for(int64_t e=graph.offsets[a];e<graph.offsets[a+1];++e) {
      const int64_t b=graph.target[e];const double next=value+graph.costs[e];
      if(next<distance[b]){distance[b]=next;queue.emplace(next,b);}
      }
    }
  }
  return 0;
}

// CSR A*, with lazy geometric edge/node invalidation supplied by the caller.
extern "C" int se2_astar_graphs(int64_t n,const Graph* graphs,int64_t count,const uint8_t* blocked_nodes,
    const double* heuristic,int64_t start,const uint8_t* goals,int64_t* path,
    int64_t* length,double* total,int64_t* expanded) {
  if(n<=0 || start<0 || start>=n || blocked_nodes[start])return 1;
  using Entry=std::pair<double,int64_t>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  std::vector<double> distance(n,std::numeric_limits<double>::infinity());
  std::vector<int64_t> parent(n,-1);
  std::vector<uint8_t> closed(n,0);
  distance[start]=0.;queue.emplace(heuristic[start],start);*expanded=0;
  while(!queue.empty()) {
    auto [priority,a]=queue.top();queue.pop();
    if(closed[a])continue;
    if(priority>distance[a]+heuristic[a]+1e-8)continue;
    if(goals[a]) {
      *total=distance[a];*length=0;
      for(int64_t node=a;node>=0;node=parent[node])path[(*length)++]=node;
      std::reverse(path,path+*length);return 0;
    }
    closed[a]=1;++*expanded;
    for(int64_t g=0;g<count;++g) {
      const auto& graph=graphs[g];if(a>=graph.nodes)continue;
      for(int64_t e=graph.offsets[a];e<graph.offsets[a+1];++e) {
      const int64_t b=graph.target[e];
      if((graph.blocked && graph.blocked[e]) || blocked_nodes[b])continue;
      const double value=distance[a]+graph.costs[e];
      if(value+1e-10<distance[b]) {
        distance[b]=value;parent[b]=a;closed[b]=0;
        queue.emplace(value+heuristic[b],b);
      }
      }
    }
  }
  return 1;
}

// Keep the single-CSR ABI for existing offline tools.
extern "C" int se2_astar(int64_t n,const int64_t* offsets,const int64_t* target,
    const double* costs,const uint8_t* blocked_edges,const uint8_t* blocked_nodes,
    const double* heuristic,int64_t start,const uint8_t* goals,int64_t* path,
    int64_t* length,double* total,int64_t* expanded) {
  const Graph graph{n,offsets,target,costs,blocked_edges};
  return se2_astar_graphs(n,&graph,1,blocked_nodes,heuristic,start,goals,path,length,total,expanded);
}
