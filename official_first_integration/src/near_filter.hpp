#pragma once
#include "scan_keys.hpp"
namespace first {
struct NearSupportScratch {
 struct Chain {size_t head=0,tail=0;std::array<size_t,3> ids{};int count=0;};
 ScanKeys<Chain> cells;std::vector<size_t> next;
};
// Spatial support on deskewed returns BEFORE voxel deduplication. Limit this
// filter to dense near-range data; do not reject distant sparse objects.
inline std::vector<uint8_t> near_support(const std::vector<std::pair<Point,Point>>& rays,
                                        NearSupportScratch& scratch,double radius=.04,double range=1.5){
 auto distance=[](Point a,Point b){return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);};
 auto bin=[&](Point p){return Key{int(std::floor(p.x/radius)),int(std::floor(p.y/radius)),int(std::floor(p.z/radius))};};
 const size_t end=rays.size();auto&cells=scratch.cells;auto&next=scratch.next;
 cells.reset(end);next.resize(end);
 for(size_t i=0;i<end;++i)if(distance(rays[i].first,rays[i].second)<=(range+radius)*(range+radius)){
  auto [cell,added]=cells.insert(bin(rays[i].second));
  if(added)cell->head=i;else next[cell->tail]=i;
  cell->tail=i;next[i]=end;
  // Keep separated witnesses in the already-built FIFO bin, avoiding a
  // second fine-grid hash insertion and lookup for every raw return.
  if(cell->count<3){bool distinct=true;
   for(int j=0;j<cell->count;++j)if(distance(rays[i].second,rays[cell->ids[j]].second)<4.000001e-6)distinct=false;
   if(distinct)cell->ids[cell->count++]=i;
  }
 }
 std::vector<uint8_t> keep(rays.size(),1);
 for(size_t i=0;i<rays.size();++i){
  auto [origin,p]=rays[i];if(distance(origin,p)>range*range)continue;
  auto k=bin(p);auto own=cells.find(k);int count=0;
  for(int j=0;j<own->count;++j){double d=distance(p,rays[own->ids[j]].second);if(d>=1e-6&&d<=radius*radius)++count;}
  // Two valid witnesses separated by >2 mm prove acceptance for any FIFO
  // first neighbor: it cannot be within 1 mm of both. Failed proofs retain
  // the original exact search and first-neighbor order below.
  if(count>=2)continue;
  count=0;Point first{};
  for(int x=-1;x<=1&&count<2;++x)for(int y=-1;y<=1&&count<2;++y)for(int z=-1;z<=1&&count<2;++z){
   auto cell=cells.find({k.x+x,k.y+y,k.z+z});if(!cell)continue;
   // FIFO order is identical to the original per-bin vector, including which
   // point is selected as the first distinct supporting neighbor.
   for(size_t j=cell->head;j!=end;j=next[j]){auto q=rays[j].second;double d=distance(p,q);
    if(d<1e-6||d>radius*radius||(count&&distance(q,first)<1e-6))continue;
    if(!count)first=q;
    if(++count==2)break;
   }
  }
  keep[i]=count>=2;
 }
 return keep;
}
inline std::vector<uint8_t> near_support(const std::vector<std::pair<Point,Point>>& rays,
                                        double radius=.04,double range=1.5){
 NearSupportScratch scratch;return near_support(rays,scratch,radius,range);
}
}
