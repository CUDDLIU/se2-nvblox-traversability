// Compare upstream ray traversal with independent analytic segment/box tests.
#include <plan_env/raycast.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
using Cell=std::array<int,3>;
void check(Eigen::Vector3d a,Eigen::Vector3d b){
  RayCaster caster;caster.setInput(a,b);Eigen::Vector3d p;std::set<Cell> got,want;
  int count=0;
  while(caster.step(p)){assert(++count<1000);assert(got.insert({int(p.x()),int(p.y()),int(p.z())}).second);}
  caster.setInput(a,b);Eigen::Vector3i ip;std::set<Cell> integer;count=0;
  while(caster.step(ip)){assert(++count<1000);assert(integer.insert({ip.x(),ip.y(),ip.z()}).second);}
  assert(integer==got);
  Cell end={int(std::floor(b.x())),int(std::floor(b.y())),int(std::floor(b.z()))};
  Eigen::Vector3d d=b-a;
  for(int x=int(std::floor(std::min(a.x(),b.x())))-1;x<=int(std::floor(std::max(a.x(),b.x())))+1;++x)
   for(int y=int(std::floor(std::min(a.y(),b.y())))-1;y<=int(std::floor(std::max(a.y(),b.y())))+1;++y)
    for(int z=int(std::floor(std::min(a.z(),b.z())))-1;z<=int(std::floor(std::max(a.z(),b.z())))+1;++z){
      Cell c={x,y,z};if(c==end)continue;double lo=0,hi=1;
      for(int k=0;k<3;++k){
        if(d[k]==0){if(int(std::floor(a[k]))!=c[k])hi=-1;}
        else{double t0=(c[k]-a[k])/d[k],t1=(c[k]+1-a[k])/d[k];lo=std::max(lo,std::min(t0,t1));hi=std::min(hi,std::max(t0,t1));}
      }
      if(hi-lo>1e-10)want.insert(c);
    }
  if(got!=want){std::cerr<<"ray mismatch "<<a.transpose()<<" -> "<<b.transpose()<<" got "<<got.size()<<" expected "<<want.size()<<std::endl;std::abort();}
}
int main(){
  check({5.136,-8.026,2.0000001},{0.,-10.,2.});
  check({3.,3.,3.},{-2.,-2.,-2.});check({-2.,-2.,-2.},{3.,3.,3.});
  check({2.,.2,.3},{-2.,.2,.3});check({.1,.2,.3},{.2,.3,.4});
  check({1.,2.,3.},{1.,2.,3.});check({.1,.2,.3},{.1,.2,4.});
  std::mt19937 rng(7);std::uniform_real_distribution<double> sample(-6.,6.);
  for(int i=0;i<500;++i){Eigen::Vector3d a(sample(rng),sample(rng),sample(rng)),b(sample(rng),sample(rng),sample(rng));check(a,b);check(b,a);}
  std::cout<<"1007 analytic ray/voxel cases passed\n";
}
