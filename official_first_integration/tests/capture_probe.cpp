#include "world.hpp"
#include <iostream>
int main(int argc,char**argv){if(argc!=4)return 2;first::Library lib(argv[1]);first::Terrain terrain;terrain.load(argv[2]);std::ifstream f(argv[3]);first::Pose base,goal;f>>base.x>>base.y>>base.z>>base.yaw>>goal.x>>goal.y;std::vector<first::Point> obs;std::vector<int> cells;auto g=terrain.at(base.x,base.y,base.z,-1,.16);double reference_offset=g?base.z-g->z:0.;int component=g?g->component:-1;double c=std::cos(base.yaw),s=std::sin(base.yaw);first::Point q;
while(f>>q.x>>q.y>>q.z){auto support=terrain.at(q.x,q.y,base.z-reference_offset,component,.8);if(q.z<=(support?support->z+reference_offset:base.z)+.1)continue;if(q.z<base.z-.25||q.z>base.z+1.4)continue;obs.push_back(q);double x=q.x-base.x,y=q.y-base.y;cells.push_back(first::Library::cell(c*x+s*y,-s*x+c*y));}
auto penalties=lib.penalties(cells);int nofloor=0,footprint=0,collision=0;auto check=[&](auto&p){double prev=base.z;for(auto pose:first::sweep(p,base)){auto floor=terrain.at(pose.x,pose.y,prev-reference_offset,component);pose.z=floor?floor->z+reference_offset:prev;for(auto o:obs)if(first::intersects(pose,o,.9)){++collision;return false;}prev=pose.z;}return true;};
auto result=first::select(lib,goal,penalties,{},check);
std::cout<<"{\"id\":"<<result.id<<",\"vx\":"<<result.command.x<<",\"vy\":"<<result.command.y<<",\"wz\":"<<result.command.w<<",\"obstacles\":"<<obs.size()<<",\"hard\":"<<result.hard_rejected<<",\"floor\":"<<nofloor<<",\"footprint\":"<<footprint<<",\"collision\":"<<collision<<"}\n";
std::sort(obs.begin(),obs.end(),[&](auto a,auto b){return std::hypot(a.x-base.x,a.y-base.y)<std::hypot(b.x-base.x,b.y-base.y);});
std::cout<<"{\"base\":["<<base.x<<","<<base.y<<","<<base.z<<"],\"nearest_obstacles\":[";
for(size_t i=0;i<std::min(size_t(20),obs.size());++i){auto p=obs[i];double dx=p.x-base.x,dy=p.y-base.y;auto support=terrain.at(p.x,p.y,base.z-reference_offset,component,.8);int cell=first::Library::cell(c*dx+s*dy,-s*dx+c*dy);if(i)std::cout<<",";std::cout<<"{\"body_xy\":["<<c*dx+s*dy<<","<<-s*dx+c*dy<<"],\"z\":"<<p.z<<",\"reference_z\":"<<(support?support->z+reference_offset:base.z)<<",\"cell\":"<<cell<<"}";}
std::cout<<"]}\n";
}
