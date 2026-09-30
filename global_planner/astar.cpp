#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

// CSR A*, with lazy geometric edge/node invalidation supplied by the caller.
extern "C" int se2_astar(int64_t n,const int64_t* offsets,const int64_t* target,
    const double* costs,const uint8_t* blocked_edges,const uint8_t* blocked_nodes,
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
    for(int64_t e=offsets[a];e<offsets[a+1];++e) {
      const int64_t b=target[e];
      if(blocked_edges[e] || blocked_nodes[b])continue;
      const double value=distance[a]+costs[e];
      if(value+1e-10<distance[b]) {
        distance[b]=value;parent[b]=a;closed[b]=0;
        queue.emplace(value+heuristic[b],b);
      }
    }
  }
  return 1;
}
