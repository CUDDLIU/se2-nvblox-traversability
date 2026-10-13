#include "first.hpp"
#include "route.hpp"
#include "terminal.hpp"
#include <iostream>
#include <chrono>
#define REQUIRE(v) do{if(!(v))throw std::runtime_error(#v);}while(0)
int main(int argc,char**argv){
 REQUIRE(argc==2);first::Library lib(argv[1]);
 REQUIRE(lib.paths[1215].velocity.x==0);REQUIRE(lib.paths[1215].velocity.w==0);
 for(const auto&p:lib.paths){double x=0,y=0;for(int i=0;i<10;++i){double a=i*.1*p.velocity.w;x+=.1*(std::cos(a)*p.velocity.x-std::sin(a)*p.velocity.y);y+=.1*(std::sin(a)*p.velocity.x+std::cos(a)*p.velocity.y);REQUIRE(std::hypot(x-p.samples[i].x,y-p.samples[i].y)<.00002);}}
 auto h=lib.penalties({});auto out=first::select(lib,{2.5,0,0,0},h,{},[](auto&){return true;});
 REQUIRE(out.id>=0);REQUIRE(out.command.x>=1.4);REQUIRE(out.command.y==0);REQUIRE(out.command.w==0);
 bool duplicate=false;
 for(int cell=0;cell<first::cells&&!duplicate;++cell){std::array<int,first::count> counts{};for(int id:lib.table[0][cell])++counts[id];
  for(int id=0;id<first::count;++id)if(counts[id]>1){auto hit=lib.penalties({cell,cell});REQUIRE(hit.hard[id]==counts[id]);duplicate=true;break;}}
 REQUIRE(duplicate);
 for(int layer=1;layer<4;++layer){int cell=first::Library::cell(1.,0.);auto hit=lib.penalties({cell});for(int id:lib.table[layer][cell])REQUIRE(hit.soft[layer-1][id]);}
 auto all=lib.penalties({first::Library::cell(0,0)});REQUIRE(first::select(lib,{2.5,0,0,0},all,{},[](auto&){return true;}).id==-1);
 auto stairs=first::select(lib,{2.5,0,0,0},h,{.4,.3,.4,.4},[](auto&){return true;});REQUIRE(stairs.command.x<=.400001);
 REQUIRE(first::select(lib,{2.5,0,0,0},h,{},[](auto&){return false;}).id==-1);
 // Preserve native gentle curves; arbitrary per-axis minimum filters erase them.
 bool gentle_seen=false;first::select(lib,{1.5,0,0,0},h,{},[&](auto&p){if(p.velocity.x==1.5&&p.velocity.w==.1)gentle_seen=true;return false;});REQUIRE(gentle_seen);
 // Find an actual native decision boundary and oscillate the goal by 1 mm
 // across it. Continuity must suppress switching without synthesizing speeds.
 double boundary=0;bool found=false;int last_id=-1;
 for(int i=0;i<=500&&!found;++i){double y=.001*i;
  auto a=first::select(lib,{2.5,y,0,0},h,{},[](auto&){return true;});
  if(last_id>=0&&a.id!=last_id){boundary=y;found=true;}last_id=a.id;
 }REQUIRE(found);
 int base_switches=0,smooth_switches=0,base_last=-1,smooth_last=-1;first::Velocity prev{};
 for(int i=0;i<40;++i){first::Pose goal{2.5,boundary-(i%2?.001:0.),0,0};
  auto a=first::select(lib,goal,h,{},[](auto&){return true;});
  auto b=first::select(lib,goal,h,{},[](auto&){return true;},i?&prev:nullptr);
  if(i){base_switches+=a.id!=base_last;smooth_switches+=b.id!=smooth_last;}
  REQUIRE(first::score(lib.paths[b.id],goal,h)<=a.score+.080001);
  REQUIRE(b.command.x==lib.paths[b.id].velocity.x&&b.command.y==lib.paths[b.id].velocity.y&&b.command.w==lib.paths[b.id].velocity.w);
  base_last=a.id;smooth_last=b.id;prev=b.command;
 }
 REQUIRE(base_switches==39);REQUIRE(smooth_switches==0);
 auto blocked_previous=h;blocked_previous.hard[smooth_last]=1;
 auto changed=first::select(lib,{2.5,boundary,0,0},blocked_previous,{},[](auto&){return true;},&prev);
 REQUIRE(changed.id>=0&&changed.id!=smooth_last);
 changed=first::select(lib,{2.5,boundary,0,0},h,{},[&](auto&p){return p.id!=smooth_last;},&prev);
 REQUIRE(changed.id>=0&&changed.id!=smooth_last);
 REQUIRE(first::select(lib,{2.5,0,0,0},all,{},[](auto&){return true;},&prev).id==-1);
 REQUIRE(first::select(lib,{2.5,0,0,0},h,{},[](auto&){return false;},&prev).id==-1);
 auto smooth_stairs=first::select(lib,{2.5,0,0,0},h,{.4,.3,.4,.4},[](auto&){return true;},&prev);
 REQUIRE(smooth_stairs.command.x<=.400001&&std::abs(smooth_stairs.command.w)<=.400001);
 auto turn=first::select(lib,{.4,2.5,0,0},h,{},[](auto&){return true;},&prev);
 REQUIRE(turn.id!=smooth_last); // A real corner overrides the near-tie bias.
 first::Route route;route.reset({{0,0,0,0},{1,0,0,0},{2,0,.15,0},{3,0,.3,0},{4,0,.3,0}});auto target=route.target({0,0,0,0});REQUIRE(std::abs(target.stairs_ahead-1)<1e-6);REQUIRE(!target.on_stairs);target=route.target({1.1,0,.015,0});REQUIRE(target.on_stairs);target=route.target({3.8,0,.3,0});REQUIRE(!target.on_stairs);
 std::vector<first::Pose> dense;for(int i=0;i<=80;++i){double x=i*.05;dense.push_back({x,0,std::clamp((x-1.5)*.5,0.,.15),0});}
 first::Route dense_route;dense_route.reset(dense);auto dt=dense_route.target({0,0,0,0});REQUIRE(dt.stairs_ahead<1.6);
 dt=dense_route.target({1.6,0,.05,0});REQUIRE(dt.on_stairs);
 dt=dense_route.target({2.9,0,.15,0});REQUIRE(!dt.on_stairs);
 // Recorded flat route: one 12 cm mesh spike returns to the same floor in
 // adjacent 5-7 cm samples. It must not request stairs metres in advance.
 std::vector<first::Pose> flat={{0,0,-.60,0},{1,0,-.61,0},{1.07,0,-.49,0},{1.12,0,-.61,0},{2,0,-.61,0}};
 first::Route noisy_flat;noisy_flat.reset(flat);REQUIRE(std::isinf(noisy_flat.target({0,0,-.60,0}).stairs_ahead));
 // A genuine short tread / isolated step stays detectable in both directions.
 for(double direction:{-1.,1.}){std::vector<first::Pose> step;
  for(int i=0;i<=60;++i){double x=i*.05;step.push_back({x,0,direction*((x>=1&&x<1.3)?.15:0),0});}
  first::Route r;r.reset(step);REQUIRE(std::isfinite(r.target({0,0,0,0}).stairs_ahead));
 }
 first::Epoch e;auto token=e.value;e.invalidate(first::State::Cancelled);REQUIRE(!e.accept(token));
 auto approach=first::terminal({1,0,0},{.11,0,0,0},.11,{});REQUIRE(approach.phase==first::Phase::Approach);REQUIRE(approach.velocity.x>=.25&&approach.velocity.x<.256);
 auto diagonal=first::terminal({1,0,0},{.15,-.12,0,0},.2,{});REQUIRE(std::abs(diagonal.velocity.y/diagonal.velocity.x+.8)<1e-9);
 auto align=first::terminal({1,0,0},{.01,0,0,-.2},.01,{});REQUIRE(align.phase==first::Phase::Align);REQUIRE(align.velocity.x==0&&align.velocity.w==-.7);
 auto capped=first::terminal({1,0,0},{0,0,0,2},0,{.4,.3,.4,.4});REQUIRE(capped.velocity.w==.4);
 REQUIRE(first::terminal({1,0,0},{0,0,0,.01},0,{}).phase==first::Phase::Arrived);
 REQUIRE(first::terminal({1,0,0},{.1,0,0,0},4,{}).phase==first::Phase::Follow);
 REQUIRE(first::terminal({1,0,0},{.1,0,3,0},.1,{}).phase==first::Phase::Follow);
 auto terminal_path=first::command_primitive({.25,0,.7});REQUIRE(std::abs(terminal_path.end.yaw-.7)<1e-9);REQUIRE(terminal_path.end.y>0);
 first::Gait gait;gait.observe(first::Gait::flat,true);REQUIRE(gait.request(first::Gait::stairs,17,0)==first::Gait::stairs);REQUIRE(!gait.may_enter_stairs());REQUIRE(gait.request(first::Gait::flat,17,1)==0);gait.observe(first::Gait::stairs,false);REQUIRE(!gait.may_enter_stairs());gait.observe(first::Gait::stairs,true);REQUIRE(gait.may_enter_stairs());
 first::Gait failed;REQUIRE(failed.request(first::Gait::stairs,1,0)==0);failed.request(first::Gait::stairs,17,0);failed.tick(3.1);REQUIRE(failed.failed);
 auto start=std::chrono::steady_clock::now();for(int i=0;i<1000;++i)out=first::select(lib,{2.5,0,0,0},h,{},[](auto&){return true;});
 std::cout<<"PASS 4641 primitives, four tables, collision/limits/cancellation/gait; selection mean ms "<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/1000<<"\n";
}
