#pragma once
#include <algorithm>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// Compact column lookup for the bounded query window. Preserve original
// per-column order (important for ties); fall back for sparse distant input.
class ColumnIndex {
  struct Key {int64_t x,y;bool operator==(const Key& k)const{return x==k.x&&y==k.y;}};
  struct Hash {size_t operator()(const Key& k)const{return std::hash<int64_t>{}(k.x)^
      (std::hash<int64_t>{}(k.y)*0x9e3779b97f4a7c15ULL);}};
  std::unordered_map<Key,std::vector<size_t>,Hash> sparse_;
  std::vector<size_t> offsets_,indices_,empty_;
  int64_t x0_=0,y0_=0,x1_=0,y1_=0;size_t width_=0;
 public:
  using Iterator=std::vector<size_t>::const_iterator;
  struct Range {Iterator first,last;Iterator begin()const{return first;}Iterator end()const{return last;}
    bool empty()const{return first==last;}};
  ColumnIndex(size_t n,const int64_t* xy) {
    if(!n)return;
    x0_=x1_=xy[0];y0_=y1_=xy[1];
    for(size_t i=1;i<n;++i){x0_=std::min(x0_,xy[2*i]);x1_=std::max(x1_,xy[2*i]);y0_=std::min(y0_,xy[2*i+1]);y1_=std::max(y1_,xy[2*i+1]);}
    const long double nx=static_cast<long double>(x1_)-x0_+1,ny=static_cast<long double>(y1_)-y0_+1;
    if(nx*ny>1000000.){
      for(size_t i=0;i<n;++i)sparse_[{xy[2*i],xy[2*i+1]}].push_back(i);
      return;
    }
    width_=static_cast<size_t>(ny);offsets_.resize(static_cast<size_t>(nx*ny)+1,0);
    auto cell=[&](size_t i){return static_cast<size_t>(xy[2*i]-x0_)*width_+static_cast<size_t>(xy[2*i+1]-y0_);};
    for(size_t i=0;i<n;++i)++offsets_[cell(i)+1];
    std::partial_sum(offsets_.begin(),offsets_.end(),offsets_.begin());
    auto next=offsets_;indices_.resize(n);
    for(size_t i=0;i<n;++i)indices_[next[cell(i)]++]=i;
  }
  Range get(int64_t x,int64_t y)const {
    if(width_){
      if(x<x0_ || x>x1_ || y<y0_ || y>y1_)return {empty_.begin(),empty_.end()};
      const size_t cell=static_cast<size_t>(x-x0_)*width_+static_cast<size_t>(y-y0_);
      return {indices_.begin()+offsets_[cell],indices_.begin()+offsets_[cell+1]};
    }
    auto it=sparse_.find({x,y});
    return it==sparse_.end()?Range{empty_.begin(),empty_.end()}:Range{it->second.begin(),it->second.end()};
  }
};
