#include "near_filter.hpp"
#include <iostream>
#include <random>
#define REQUIRE(v) do{if(!(v))throw std::runtime_error(#v);}while(0)

// Deliberately keep the allocation-heavy original as an independent oracle.
std::vector<uint8_t> reference(const std::vector<std::pair<first::Point,first::Point>>&rays,double radius,double range){
 using namespace first;
 auto distance=[](Point a,Point b){return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);};
 auto bin=[&](Point p){return Key{int(std::floor(p.x/radius)),int(std::floor(p.y/radius)),int(std::floor(p.z/radius))};};
 std::unordered_map<Key,std::vector<size_t>,Hash> cells;
 for(size_t i=0;i<rays.size();++i)if(distance(rays[i].first,rays[i].second)<=(range+radius)*(range+radius))cells[bin(rays[i].second)].push_back(i);
 std::vector<uint8_t> keep(rays.size(),1);
 for(size_t i=0;i<rays.size();++i){auto [origin,p]=rays[i];if(distance(origin,p)>range*range)continue;
  auto k=bin(p);int count=0;Point first{};
  for(int x=-1;x<=1&&count<2;++x)for(int y=-1;y<=1&&count<2;++y)for(int z=-1;z<=1&&count<2;++z){
   auto it=cells.find({k.x+x,k.y+y,k.z+z});if(it==cells.end())continue;
   for(auto j:it->second){auto q=rays[j].second;double d=distance(p,q);
    if(d<1e-6||d>radius*radius||(count&&distance(q,first)<1e-6))continue;
    if(!count)first=q;
    if(++count==2)break;
   }
  }keep[i]=count>=2;
 }return keep;
}

int main(){
 using namespace first;Point origin{0,0,0};std::vector<std::pair<Point,Point>> rays;
 rays.push_back({origin,{.5,0,.2}}); // Isolated near return.
 for(int i=0;i<7;++i)rays.push_back({origin,{.8,.2,.01*i}}); // Thin vertical object.
 rays.push_back({origin,{2.,0,.2}}); // Distant singleton is outside filter scope.
 auto keep=near_support(rays);REQUIRE(!keep[0]);for(size_t i=1;i<keep.size();++i)REQUIRE(keep[i]);
 rays={{{0,0,0},{.5,0,.2}},{{0,0,0},{.51,0,.2}},{{0,0,0},{.51,0,.2}}};
 keep=near_support(rays);REQUIRE(!keep[0]); // Duplicated coordinates are not independent support.
 Evidence grid;Point obstacle{.5,0,.2};grid.ingest({{origin,obstacle}},origin,1.);
 std::vector<std::pair<Point,Point>> accepted;
 for(size_t i=0;i<rays.size();++i)if(keep[i])accepted.push_back(rays[i]);
 grid.ingest(accepted,origin,2.);REQUIRE(grid.state(obstacle)==1); // Rejection never ray-clears old evidence.
 NearSupportScratch scratch;std::mt19937 random(702);std::uniform_real_distribution<double> u(-2.,2.);
 for(int iteration=0;iteration<100;++iteration){
  rays.clear();size_t n=iteration%10==0?0:iteration%7==0?5000:237;
  for(size_t i=0;i<n;++i){Point o{i%2?.32028:-.32028,0,-.013};Point p{u(random),u(random),u(random)};
   if(i%3==0)p={.5+.004*(i%20),-.1+.004*((i/20)%20),.2};
   if(i&&i%5==0)p=rays.back().second;
   if(i&&i%11==0){p=rays.back().second;p.x+=.000999;}
   if(i%17==0)p={o.x+(i%2?1.5:1.54),0,o.z};
   rays.push_back({o,p});
  }
  REQUIRE(near_support(rays,scratch)==reference(rays,.04,1.5));
  REQUIRE(near_support(rays,scratch,.025,.9)==reference(rays,.025,.9));
 }
 ScanKeys<int> growing;growing.reset(1);
 for(int i=0;i<20000;++i)*growing.insert({i,-i,i%31}).first=i+7;
 for(int i=0;i<20000;++i)REQUIRE(*growing.find({i,-i,i%31})==i+7);
 growing.reset(1);REQUIRE(!growing.find({1,-1,1}));
 ScanKeys<uint8_t> keys;
 for(int round=0;round<20;++round){size_t n=round%2?10000:5;keys.reset(n);std::unordered_set<Key,Hash> expected;
  for(size_t i=0;i<n;++i){Key k{int(random()%173)-86,int(random()%101)-50,int(random()%47)-23};
   REQUIRE(keys.insert(k).second==expected.insert(k).second);REQUIRE(keys.find(k));}
  for(auto k:expected)REQUIRE(!keys.insert(k).second);
  keys.reset(0);for(auto k:expected)REQUIRE(!keys.find(k));
 }
 std::cout<<"PASS isolated near return, thin object, distant singleton, duplicate support, no false clearing\n";
}
