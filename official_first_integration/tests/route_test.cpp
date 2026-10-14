#include "route.hpp"
#include <iostream>
#define REQUIRE(v) do{if(!(v))throw std::runtime_error(#v);}while(0)
int main(){
 using namespace first;
 // Doorway at (2,0): first leave the room, then turn. Yaw-only duplicates
 // used to hide this bend and feed a goal on the far side of the wall.
 Route r;r.reset({{0,0,0,0},{1,0,0,0},{2,0,0,0},{2,0,0,.7},{2,0,0,pi/2},{2,1,0,pi/2},{2,3,0,pi/2}});
 auto t=r.target({.5,0,0,0});REQUIRE(t.goal.x<=2.000001&&t.goal.y<.15);
 t=r.target({1.5,0,0,0});REQUIRE(t.goal.y<.15);REQUIRE(std::abs(t.heading)<1e-9);
 t=r.target({1.8,0,0,0});REQUIRE(std::abs(t.heading-pi/2)<1e-9);
 t=r.target({2,0,0,pi/2},1.5);REQUIRE(t.goal.y>2.&&t.goal.x==2.);REQUIRE(std::abs(t.heading-pi/2)<1e-9);
 // Several sub-60-degree bends must not permit a diagonal through a corner.
 r.reset({{0,0,0,0},{1,0,0,0},{1.2,.1,0,.4},{1.4,.3,0,.8},{1.5,.5,0,1.2},{1.5,3,0,pi/2}});
 t=r.target({.5,0,0,0});REQUIRE(t.goal.y<.3);
 // Deviating sideways asks for the nearby route, not a point around a bend.
 r.reset({{0,0,0,0},{2,0,0,0},{2,2,0,pi/2}});
 t=r.target({.5,-.4,0,0});REQUIRE(t.rejoining&&t.goal.y<.15);
 t=r.target({.5,-.3,0,0});REQUIRE(t.rejoining);
 t=r.target({.5,-.1,0,0});REQUIRE(!t.rejoining);
 // A straight route retains full lookahead and a small offset is not rejoin.
 r.reset({{0,0,0,0},{1,0,0,0},{1,0,0,.1},{5,0,0,0}});
 t=r.target({.5,.05,0,0},1.5);REQUIRE(!t.rejoining&&std::abs(t.goal.x-3.)<1e-9);
 // Speed stretches straight preview; heading error and a bend shorten it.
 r.reset({{0,0,0,0},{5,0,0,0}});
 auto slow=r.target({.5,0,0,0},0),fast=r.target({.5,0,0,0},1.5);
 REQUIRE(slow.lookahead<fast.lookahead&&std::abs(slow.lookahead-1.)<1e-9);
 auto misaligned=r.target({.5,0,0,pi/2},1.5);REQUIRE(misaligned.lookahead<fast.lookahead);REQUIRE(std::abs(misaligned.heading)<1e-9);
 r.reset({{0,0,0,0},{2,0,0,0},{2,0,0,pi/2},{2,3,0,pi/2}});
 auto bend=r.target({.5,0,0,0},1.5);REQUIRE(bend.lookahead<fast.lookahead&&bend.goal.y<.15);
 // Small saved-mesh zigzags on an otherwise straight corridor must not
 // suppress the straight-line start preview after a sensor hold.
 std::vector<Pose> jitter;for(int i=0;i<=30;++i)jitter.push_back({i*.1,i%2?.025:-.025,0,0});
 r.reset(jitter);auto restart=r.target({.5,0,0,0},0);REQUIRE(restart.lookahead>.75);
 std::cout<<"PASS doorway, yaw duplicates, distributed turn, rejoin and straight tracking\n";
}
