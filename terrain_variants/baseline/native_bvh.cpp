#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <limits>
#include <map>
#include <set>
#include <memory>
#include <atomic>
#include <omp.h>
#include "raster_types.h"

extern "C" int raster(const double*,size_t,const int64_t*,size_t,double,double,Record**,size_t*);
extern "C" int merge_raster_records(const Record*,size_t,double,double,SpanRecord**,size_t*);
extern "C" int merge_mixed_records(const Record*,size_t,const SpanRecord*,size_t,double,double,SpanRecord**,size_t*);

namespace {
struct Box { std::array<double,3> lo,hi; };
struct Node { Box box; int left=-1,right=-1; size_t begin=0,end=0; };
struct Tree {
  std::vector<Box> boxes;
  std::vector<int64_t> order;
  std::vector<Node> nodes;
  size_t leaf;
  std::vector<Record> raster_cache;
  std::vector<SpanRecord> span_cache;
  double span_dz=-1.;
  double raster_resolution=-1.,raster_slope=-2.;
  Tree(const double* lo,const double* hi,size_t n,size_t leaf_):boxes(n),order(n),leaf(leaf_) {
    if(!leaf) throw std::runtime_error("leaf size");
    for(size_t i=0;i<n;++i) for(int d=0;d<3;++d) {
      if(!std::isfinite(lo[3*i+d]) || !std::isfinite(hi[3*i+d]) || lo[3*i+d]>hi[3*i+d])
        throw std::runtime_error("bounds");
      boxes[i].lo[d]=lo[3*i+d];boxes[i].hi[d]=hi[3*i+d];
    }
    std::iota(order.begin(),order.end(),0);
    nodes.reserve(n*2);
    if(n) build(0,n);
  }
  int build(size_t begin,size_t end) {
    Node node;node.begin=begin;node.end=end;node.box=boxes[order[begin]];
    std::array<double,3> cl=boxes[order[begin]].lo,ch=cl;
    for(size_t j=begin;j<end;++j) for(int d=0;d<3;++d) {
      auto& b=boxes[order[j]];
      node.box.lo[d]=std::min(node.box.lo[d],b.lo[d]);node.box.hi[d]=std::max(node.box.hi[d],b.hi[d]);
      double c=(b.lo[d]+b.hi[d])*.5;
      if(j==begin)cl[d]=ch[d]=c;else{cl[d]=std::min(cl[d],c);ch[d]=std::max(ch[d],c);}
    }
    int id=static_cast<int>(nodes.size());nodes.push_back(node);
    if(end-begin>leaf) {
      int axis=0;for(int d=1;d<3;++d) if(ch[d]-cl[d]>ch[axis]-cl[axis])axis=d;
      size_t middle=(begin+end)/2;
      std::nth_element(order.begin()+begin,order.begin()+middle,order.begin()+end,[&](int64_t a,int64_t b){
        double ca=boxes[a].lo[axis]+boxes[a].hi[axis],cb=boxes[b].lo[axis]+boxes[b].hi[axis];
        return ca==cb?a<b:ca<cb;
      });
      int left=build(begin,middle),right=build(middle,end);
      nodes[id].left=left;nodes[id].right=right;
    }
    return id;
  }
  static bool hit(const Box& b,const double* lo,const double* hi) {
    return b.hi[0]>=lo[0]&&b.lo[0]<=hi[0]&&b.hi[1]>=lo[1]&&b.lo[1]<=hi[1]&&b.hi[2]>=lo[2]&&b.lo[2]<=hi[2];
  }
  void query(const double* lo,const double* hi,std::vector<int64_t>& found,size_t& visits,bool first) const {
    std::vector<int> stack;if(!nodes.empty())stack.push_back(0);
    while(!stack.empty()) {
      const auto& node=nodes[stack.back()];stack.pop_back();++visits;
      if(!hit(node.box,lo,hi))continue;
      if(node.left<0) {
        for(size_t j=node.begin;j<node.end;++j) if(hit(boxes[order[j]],lo,hi)) {
          found.push_back(order[j]);if(first)return;
        }
      } else {stack.push_back(node.right);stack.push_back(node.left);}
    }
    std::sort(found.begin(),found.end());
  }
};
}
extern "C" void* bvh_create(const double* lo,const double* hi,size_t n,size_t leaf) {
  try{return new Tree(lo,hi,n,leaf);}catch(...){return nullptr;}
}
extern "C" void bvh_destroy(void* tree){delete static_cast<Tree*>(tree);}
extern "C" int bvh_query(void* tree,const double* lo,const double* hi,int first,
                          int64_t** out,size_t* n,size_t* visits) {
  *out=nullptr;*n=0;*visits=0;
  try {
    if(!tree)throw std::runtime_error("null tree");
    std::vector<int64_t> ids;static_cast<Tree*>(tree)->query(lo,hi,ids,*visits,first!=0);
    if(!ids.empty()) {
      *out=static_cast<int64_t*>(std::malloc(ids.size()*sizeof(int64_t)));
      if(!*out)throw std::bad_alloc();
      std::memcpy(*out,ids.data(),ids.size()*sizeof(int64_t));
    }
    *n=ids.size();return 0;
  } catch(...) {std::free(*out);*out=nullptr;return -1;}
}
extern "C" void bvh_free_indices(int64_t* indices){std::free(indices);}

// Batch both BVH triangle selection and cached raster assembly to avoid a
// Python/ctypes transition for every block in each neighboring tile's halo.
extern "C" int bvh_mesh_spans(size_t count, void* const* trees,
    const double* const* vertices, const int64_t* const* triangles,
    const size_t* nv, const size_t* nt, const double* low, const double* high,
    double resolution, double dz, double slope, double height,
    SpanRecord** out,size_t* size,size_t* stats) {
  *out=nullptr;*size=0;
  std::fill(stats,stats+3,0);
  try {
    struct Piece {std::vector<Record> records;std::vector<SpanRecord> spans;
                  size_t visits=0,triangles=0,misses=0;};
    std::vector<Piece> pieces(count);
    std::atomic<bool> failed{false};
    #pragma omp parallel for schedule(dynamic,4) num_threads(std::min(3,omp_get_max_threads())) if(count>16)
    for(size_t i=0;i<count;++i) {
      try {
      auto& piece=pieces[i];
      Tree& tree=*static_cast<Tree*>(trees[i]);
      if(tree.nodes.empty())continue;
      const Box& bounds=tree.nodes[0].box;
      bool contained=true;
      for(int d=0;d<3;++d) contained &= bounds.lo[d]>=low[d] && bounds.hi[d]<=high[d];
      std::vector<int64_t> ids;
      if(!contained) tree.query(low,high,ids,piece.visits,false);
      piece.triangles=contained?nt[i]:ids.size();
      if(!contained && ids.empty())continue;
      if(tree.raster_resolution!=resolution || tree.raster_slope!=slope) {
        Record* buffer=nullptr;size_t n=0;
        if(raster(vertices[i],nv[i],triangles[i],nt[i],resolution,slope,&buffer,&n))
          throw std::runtime_error("raster failure");
        try {
          if(n)tree.raster_cache.assign(buffer,buffer+n);else tree.raster_cache.clear();
        } catch(...) {std::free(buffer);throw;}
        std::free(buffer);
        tree.raster_resolution=resolution;tree.raster_slope=slope;++piece.misses;
        tree.span_dz=-1.;tree.span_cache.clear();
      }
      if(contained) {
        // Integer interval union and highest-top support are associative.
        // Cache each fully selected block's occupied union; ceilings/walk are
        // recomputed after the cross-block union, never used from this cache.
        if(tree.span_dz!=dz) {
          SpanRecord* buffer=nullptr;size_t n=0;
          if(merge_raster_records(tree.raster_cache.data(),tree.raster_cache.size(),dz,0.,&buffer,&n))
            throw std::runtime_error("block span merge failure");
          try {if(n)tree.span_cache.assign(buffer,buffer+n);else tree.span_cache.clear();}
          catch(...){std::free(buffer);throw;}
          std::free(buffer);tree.span_dz=dz;
        }
        piece.spans=tree.span_cache;
      }
      else {
        std::vector<uint8_t> selected(nt[i],0);
        for(int64_t id:ids)selected[id]=1;
        for(const Record& rec:tree.raster_cache) if(selected[rec.reserved]) piece.records.push_back(rec);
      }
      }catch(...){failed.store(true);}
    }
    if(failed.load())throw std::runtime_error("parallel block query failed");
    size_t nr=0,ns=0;
    for(const auto& p:pieces){nr+=p.records.size();ns+=p.spans.size();}
    std::vector<Record> records;records.reserve(nr);
    std::vector<SpanRecord> spans;spans.reserve(ns);
    for(const auto& p:pieces) {
      records.insert(records.end(),p.records.begin(),p.records.end());
      spans.insert(spans.end(),p.spans.begin(),p.spans.end());
      stats[0]+=p.visits;stats[1]+=p.triangles;stats[2]+=p.misses;
    }
    return merge_mixed_records(records.data(),records.size(),spans.data(),spans.size(),dz,height,out,size);
  } catch(...) {std::free(*out);*out=nullptr;*size=0;return -1;}
}

namespace {
using MeshKey=std::array<int64_t,3>;
struct MeshBlock {
  std::vector<double> vertices;
  std::vector<int64_t> triangles;
  std::unique_ptr<Tree> tree;
};
struct MeshStore {
  std::map<MeshKey,MeshBlock> blocks;
  std::vector<MeshBlock*> ordered;
  std::unique_ptr<Tree> top;
  bool dirty=true;
  void rebuild() {
    if(!dirty)return;
    std::vector<double> low,high;ordered.clear();
    for(auto& item:blocks) {
      ordered.push_back(&item.second);
      const Box& box=item.second.tree->nodes[0].box;
      low.insert(low.end(),box.lo.begin(),box.lo.end());
      high.insert(high.end(),box.hi.begin(),box.hi.end());
    }
    top=std::make_unique<Tree>(low.data(),high.data(),ordered.size(),4);
    dirty=false;
  }
};
void affect(const Box& box,const double* border,const double* size,std::set<MeshKey>& dirty) {
  MeshKey first,last;
  size_t volume=1;
  for(int d=0;d<3;++d) {
    first[d]=static_cast<int64_t>(std::floor((box.lo[d]-border[d])/size[d]));
    last[d]=static_cast<int64_t>(std::floor((box.hi[d]+border[d])/size[d]));
    if(last[d]-first[d]>100000)throw std::runtime_error("update extent");
    volume*=last[d]-first[d]+1;
  }
  if(volume>100000)throw std::runtime_error("update extent");
  for(int64_t x=first[0];x<=last[0];++x)for(int64_t y=first[1];y<=last[1];++y)
    for(int64_t z=first[2];z<=last[2];++z)dirty.insert({x,y,z});
}
}
extern "C" void* mesh_store_create(){try{return new MeshStore;}catch(...){return nullptr;}}
extern "C" void mesh_store_destroy(void* ptr){delete static_cast<MeshStore*>(ptr);}
extern "C" void mesh_store_clear(void* ptr){
  auto& s=*static_cast<MeshStore*>(ptr);s.blocks.clear();s.ordered.clear();s.top.reset();s.dirty=true;
}
extern "C" int mesh_store_update(void* ptr,size_t count,const int64_t* keys,
    const double* const* vertices,const int64_t* const* triangles,
    const size_t* nv,const size_t* nt,const double* border,const double* size,int reconfigure,
    int64_t** out,size_t* out_count,size_t* stats) {
  *out=nullptr;*out_count=0;std::fill(stats,stats+3,0);
  try {
    auto& store=*static_cast<MeshStore*>(ptr);
    std::set<MeshKey> dirty;
    if(reconfigure)for(auto& item:store.blocks)affect(item.second.tree->nodes[0].box,border,size,dirty);
    for(size_t i=0;i<count;++i) {
      const MeshKey key={keys[3*i],keys[3*i+1],keys[3*i+2]};
      auto old=store.blocks.find(key);
      if(old!=store.blocks.end() && old->second.vertices.size()==3*nv[i] &&
          old->second.triangles.size()==3*nt[i] &&
          std::equal(old->second.vertices.begin(),old->second.vertices.end(),vertices[i]) &&
          std::equal(old->second.triangles.begin(),old->second.triangles.end(),triangles[i])) {++stats[1];continue;}
      if(!nt[i] && old==store.blocks.end()){++stats[1];continue;}
      if(old!=store.blocks.end())affect(old->second.tree->nodes[0].box,border,size,dirty);
      if(!nt[i]){store.blocks.erase(key);store.dirty=true;++stats[0];continue;}
      MeshBlock block;
      block.vertices.assign(vertices[i],vertices[i]+3*nv[i]);
      block.triangles.assign(triangles[i],triangles[i]+3*nt[i]);
      for(double v:block.vertices)if(!std::isfinite(v))throw std::runtime_error("vertex");
      std::vector<double> low(3*nt[i]),high(3*nt[i]);
      for(size_t j=0;j<nt[i];++j)for(int d=0;d<3;++d) {
        double l=std::numeric_limits<double>::infinity(),h=-l;
        for(int v=0;v<3;++v) {
          int64_t id=triangles[i][3*j+v];
          if(id<0 || id>=static_cast<int64_t>(nv[i]))throw std::runtime_error("triangle");
          double value=vertices[i][3*id+d];l=std::min(l,value);h=std::max(h,value);
        }
        low[3*j+d]=l;high[3*j+d]=h;
      }
      block.tree=std::make_unique<Tree>(low.data(),high.data(),nt[i],8);
      affect(block.tree->nodes[0].box,border,size,dirty);
      store.blocks[key]=std::move(block);store.dirty=true;++stats[0];
    }
    if(!dirty.empty()) {
      *out=static_cast<int64_t*>(std::malloc(dirty.size()*3*sizeof(int64_t)));
      if(!*out)throw std::bad_alloc();
      size_t j=0;for(const auto& key:dirty)for(int64_t v:key)(*out)[j++]=v;
    }
    *out_count=dirty.size();stats[2]=store.blocks.size();return 0;
  }catch(...){std::free(*out);*out=nullptr;*out_count=0;return -1;}
}
extern "C" int mesh_store_has(void* ptr,const double* low,const double* high) {
  try {
    auto& s=*static_cast<MeshStore*>(ptr);s.rebuild();
    std::vector<int64_t> ids;size_t visits=0;
    s.top->query(low,high,ids,visits,false);
    for(int64_t id:ids){std::vector<int64_t> hits;s.ordered[id]->tree->query(low,high,hits,visits,true);if(!hits.empty())return 1;}
    return 0;
  }catch(...){return -1;}
}
extern "C" int mesh_store_spans(void* ptr,const double* low,const double* high,
    double resolution,double dz,double slope,double height,SpanRecord** out,size_t* count,size_t* stats) {
  *out=nullptr;*count=0;
  try {
    auto& s=*static_cast<MeshStore*>(ptr);s.rebuild();
    std::vector<int64_t> ids;size_t visits=0;
    s.top->query(low,high,ids,visits,false);
    std::vector<void*> trees;std::vector<const double*> vertices;
    std::vector<const int64_t*> triangles;std::vector<size_t> nv,nt;
    for(int64_t id:ids) {
      auto& b=*s.ordered[id];trees.push_back(b.tree.get());
      vertices.push_back(b.vertices.data());triangles.push_back(b.triangles.data());
      nv.push_back(b.vertices.size()/3);nt.push_back(b.triangles.size()/3);
    }
    return bvh_mesh_spans(ids.size(),trees.data(),vertices.data(),triangles.data(),nv.data(),nt.data(),
                           low,high,resolution,dz,slope,height,out,count,stats);
  }catch(...){std::free(*out);*out=nullptr;*count=0;return -1;}
}

// Layer connectivity uses exactly the reference's unique, reciprocal neighbor
// rule. Multiple possible floors remain blocked, even if one looks nearer.
extern "C" int span_neighbors(size_t n, const int64_t* xy, const double* z,
    const double* roof, const uint8_t* walk, const uint8_t* slope,
    double step, double height, int64_t* neighbors, uint8_t* missing,
    int32_t* reasons, int64_t* components) {
  try {
    struct Key {int64_t x,y; bool operator==(const Key& o) const {return x==o.x && y==o.y;}};
    struct Hash {size_t operator()(const Key& k) const {
      return std::hash<int64_t>{}(k.x) ^ (std::hash<int64_t>{}(k.y)*0x9e3779b97f4a7c15ULL);
    }};
    std::unordered_map<Key,std::vector<size_t>,Hash> columns;
    for(size_t i=0;i<n;++i) columns[{xy[2*i],xy[2*i+1]}].push_back(i);
    std::fill(neighbors,neighbors+(n+1)*4,n);
    std::fill(missing,missing+(n+1)*4,0);
    std::fill(components,components+n,-1);
    const int dx[]={1,-1,0,0},dy[]={0,0,1,-1};
    for(size_t i=0;i<n;++i) {
      reasons[i]=walk[i]?0:(!slope[i]?1:2);
      for(int d=0;d<4;++d) {
        auto found=columns.find({xy[2*i]+dx[d],xy[2*i+1]+dy[d]});
        missing[4*i+d]=found==columns.end();
        size_t count=0,choice=n;
        if(found!=columns.end()) for(size_t j:found->second) {
          if(walk[j] && std::abs(z[j]-z[i])<=step+1e-9 &&
             std::min(roof[i],roof[j])-std::max(z[i],z[j])>=height-1e-9) {++count;choice=j;}
        }
        if(walk[i] && count==1) neighbors[4*i+d]=choice;
        else if(walk[i]) reasons[i]=missing[4*i+d]?3:4;
      }
    }
    for(size_t i=0;i<n;++i) for(int d=0;d<4;++d) {
      size_t j=neighbors[4*i+d];
      if(j!=n && neighbors[4*j+(d^1)]!=static_cast<int64_t>(i)) {
        neighbors[4*i+d]=n;reasons[i]=5;
      }
    }
    int64_t component=0;
    for(size_t start=0;start<n;++start) {
      if(!walk[start] || components[start]>=0) continue;
      std::vector<size_t> todo{start};components[start]=component;
      for(size_t p=0;p<todo.size();++p) for(int d=0;d<4;++d) {
        size_t j=neighbors[4*todo[p]+d];
        if(j!=n && components[j]<0) {components[j]=component;todo.push_back(j);}
      }
      ++component;
    }
    return 0;
  } catch(...) {return -1;}
}
