#include "world.hpp"
#include <iostream>
#include <random>
#define REQUIRE(v) do{if(!(v))throw std::runtime_error(#v);}while(0)
int main(){
 first::Evidence grid;first::Point a{0,0,0},b{1,0,0};grid.ingest({{a,b}},a,1);REQUIRE(grid.state(b)==1);REQUIRE(grid.state({.5,0,0})==-1);REQUIRE(grid.state({0,.5,0})==0);
 grid.ingest({},a,2);REQUIRE(grid.state(b)==1); // Silence cannot clear a hit.
 grid.ingest({{a,{8,0,0}}},a,3);REQUIRE(grid.state(b)==-1);REQUIRE(grid.state({4,0,0})==-1);REQUIRE(grid.state({4.1,0,0})==0);
 grid.ingest({{a,b},{a,{2,0,0}}},a,4);REQUIRE(grid.state(b)==1); // Simultaneous hit wins.
 auto ray=first::Evidence::ray({-.01,-.01,-.01},{-.11,-.11,-.11});REQUIRE(ray.front()==first::Evidence::key({-.01,-.01,-.01}));REQUIRE(ray.back()==first::Evidence::key({-.11,-.11,-.11}));
 for(int i=0;i<360;++i){double a=i*first::pi/180;auto v=first::Evidence::ray({.32028,0,-.013},{3*std::cos(a),3*std::sin(a),-.57});REQUIRE(v.size()<300);}
 // Differential oracle: the previous dense rebuild, including window shifts,
 // repeated endpoints and rays which later clear an earlier scan's hits.
 first::Evidence fast,dense;
 for(int frame=0;frame<14;++frame){
  first::Point body{frame*.17-.9,frame*.07-.4,frame*.03-.2};
  std::vector<std::pair<first::Point,first::Point>> rays;
  for(int i=0;i<90;++i){double a=(i*13+frame*7)*first::pi/180.;double range=(i%7==0)?6.:.3+(i%23)*.12;
   rays.push_back({body,{body.x+range*std::cos(a),body.y+range*std::sin(a),body.z-.4+(i%5)*.2}});
   if(i%9==0)rays.push_back(rays.back());
  }
  fast.ingest(rays,body,frame);dense.move_window(body);
  std::vector<std::pair<first::Point,first::Point>> clipped;
  for(auto [a,b]:rays){double dx=b.x-a.x,dy=b.y-a.y,dz=b.z-a.z,d=std::sqrt(dx*dx+dy*dy+dz*dz);
   if(d<=4.){auto k=first::Evidence::key(b);if(dense.inside(k))dense.values[dense.offset(k)]=2;}
   else b={a.x+4*dx/d,a.y+4*dy/d,a.z+4*dz/d};
   clipped.push_back({a,b});
  }
  for(auto [a,b]:clipped)for(auto k:first::Evidence::ray(a,b))if(dense.inside(k)){auto&v=dense.values[dense.offset(k)];if(v!=2)v=-1;}
  size_t occupied=0;for(auto&v:dense.values)if(v>0){v=1;++occupied;}
  REQUIRE(fast.values==dense.values);REQUIRE(fast.occupied().size()==occupied);
 }
 first::Terrain terrain;for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y){terrain.cells[{x,y,0}]={{0,0},{3,1}};}
 REQUIRE(terrain.at(0,0,0)->component==0);REQUIRE(!terrain.at(0,0,3,0));REQUIRE(terrain.footprint({0,0,0,0},0));terrain.cells.erase({8,0,0});REQUIRE(!terrain.footprint({0,0,0,0},0));
 first::Terrain exact;exact.triangles.push_back({{{{.01,0,0},{.05,0,0},{.05,.05,0}}},7});exact.index[{0,0,0}]={0};
 REQUIRE(exact.at(.049,.001,0,7));REQUIRE(!exact.at(.001,.001,0,7));REQUIRE(!exact.at(.049,.001,3,7));
 first::Terrain cached;
 cached.triangles={{{{{-.2,-.2,0},{.2,-.2,.02},{.2,.2,.04}}},0},{{{{-.2,-.2,3},{.2,-.2,3},{.2,.2,3}}},1}};
 for(int x=-4;x<=4;++x)for(int y=-4;y<=4;++y)cached.index[{x,y,0}]={0,1};
 for(int pass=0;pass<2;++pass)for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y)
  for(double z:{-.5,0.,.5,2.9,3.,4.})for(int component:{-1,0,1,7})for(double step:{.2,.8,5.}){
   first::Key k{x,y,0};auto p=first::Evidence::center(k);
   auto direct=cached.at(p.x,p.y,z,component,step),fast=cached.at_voxel_center(k,z,component,step);
   REQUIRE(bool(direct)==bool(fast));if(direct){REQUIRE(direct->z==fast->z);REQUIRE(direct->component==fast->component);}
  }
 REQUIRE(first::intersects({0,0,0,0},{0,.26,.5},.9));REQUIRE(!first::intersects({0,0,0,0},{0,.4,.5},.9));
 // Indexed broad phase must exactly match dense collision checks, including
 // bin boundaries, signed coordinates, arbitrary yaw and strict Z bounds.
 std::mt19937 rng(847);std::uniform_real_distribution<double> xy(-5,5),z(-1,2),yaw(-first::pi,first::pi);
 std::vector<first::Point> obstacles;
 for(int i=0;i<1200;++i)obstacles.push_back({xy(rng),xy(rng),z(rng)});
 first::ObstacleIndex indexed(obstacles);
 for(int i=0;i<3000;++i){first::Pose pose{xy(rng),xy(rng),z(rng),yaw(rng)};bool dense_hit=false;
  for(auto q:obstacles)if(first::intersects(pose,q,.9)){dense_hit=true;break;}
  REQUIRE(indexed.collides(pose,.9)==dense_hit);
 }
 for(auto q:std::vector<first::Point>{{.44536,0,.5},{0,.28836,.5},{0,0,.1},{0,0,.93536},{-.5,-.5,.5}}){
  first::ObstacleIndex single({q});REQUIRE(single.collides({0,0,0,0},.9)==first::intersects({0,0,0,0},q,.9));
 }
 std::cout<<"PASS occupied/free/unknown, ray clearing, dual-ray hit priority, negative coordinates, distinct floors, support holes, actual footprint\n";
}
