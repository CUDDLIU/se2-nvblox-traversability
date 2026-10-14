#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace first {
constexpr int count=4641, cells=7200;
constexpr double pi=3.14159265358979323846;
inline double wrap(double a){return std::remainder(a,2*pi);}
struct Pose {double x=0,y=0,z=0,yaw=0;};
struct Velocity {double x=0,y=0,w=0;};
struct Primitive {int id=-1;std::array<Pose,10> samples;Pose end;Velocity velocity;};
struct Penalties {std::array<int,count> hard{};std::array<std::array<bool,count>,3> soft{};};
class Library {
 public:
 std::array<Primitive,count> paths;
 std::array<std::vector<std::vector<int>>,4> table;
 explicit Library(const std::string& dir){
  std::array<int,count> seen{};std::ifstream p(dir+"/pathList.ply");
  if(!p)throw std::runtime_error("missing pathList");
  double x,y,z;int id;
  while(p>>x>>y>>z>>id){if(id<0||id>=count||seen[id]>=10||!std::isfinite(x+y+z))throw std::runtime_error("invalid path sample");paths[id].samples[seen[id]++]={x,y,z,0};}
  if(!p.eof())throw std::runtime_error("malformed pathList");
  for(int n:seen)if(n!=10)throw std::runtime_error("incomplete samples");
  seen.fill(0);std::ifstream e(dir+"/path_end.ply");double a,vx,vy,w;
  if(!e)throw std::runtime_error("missing endpoints");
  while(e>>x>>y>>z>>a>>vx>>vy>>w>>id){
   if(id<0||id>=count||seen[id]++||!std::isfinite(x+y+z+a+vx+vy+w))throw std::runtime_error("invalid endpoint");
   auto&q=paths[id];q.id=id;q.end={x,y,z,a};q.velocity={vx,vy,w};
   for(int i=0;i<10;++i)q.samples[i].yaw=w*(i+1)*.1;
   if(std::hypot(q.samples[9].x-x,q.samples[9].y-y)>1e-5||std::abs(a-w)>1e-5)throw std::runtime_error("endpoint mismatch");
  }
  if(!e.eof())throw std::runtime_error("malformed endpoints");
  for(int n:seen)if(n!=1)throw std::runtime_error("missing endpoint");
  const char* names[]={"correspondences.ply","correspondences_4.ply","correspondences_5.ply","correspondences_6.ply"};
  for(int l=0;l<4;++l){
   table[l].resize(cells);std::ifstream f(dir+"/"+names[l]);if(!f)throw std::runtime_error("missing collision table");
   for(int row=0;row<cells;++row){int k;if(!(f>>k)||k!=row)throw std::runtime_error("table row order");
    bool end=false;while(f>>id){if(id==-1){end=true;break;}if(id<0||id>=count)throw std::runtime_error("table path ID");table[l][row].push_back(id);}
    if(!end)throw std::runtime_error("unterminated table row");
   }
   std::string trailing;if(f>>trailing)throw std::runtime_error("extra table rows");
  }
 }
 static int cell(double x,double y){
  if(!std::isfinite(x+y))return -1;
  int i=int(std::floor((x+1.475)/.05)),j=int(std::floor((y+1.975)/.05));
  return i>=0&&i<90&&j>=0&&j<80?i*80+j:-1;
 }
 Penalties penalties(const std::vector<int>& occupied)const{
  Penalties out;std::array<bool,cells> seen{};
  for(int cell:occupied){if(cell<0||cell>=cells||seen[cell])continue;seen[cell]=true;
   for(int id:table[0][cell])++out.hard[id]; // Duplicates inside official rows are meaningful.
   for(int l=1;l<4;++l)for(int id:table[l][cell])out.soft[l-1][id]=true; // Binary flags, NOT summed counts.
  }return out;
 }
};
struct Limits {double x=1.5,y=.4,w=.8,planar=1.5;};
struct Weights {double goal=1,yaw=.5,lateral=.2;std::array<double,3> obstacle={1.,.8,.6};};
struct Selection {int id=-1;double score=std::numeric_limits<double>::infinity();int hard_rejected=0,limit_rejected=0,terrain_rejected=0;Velocity command;};
enum class Direction {Any,Forward,Reverse};
// Project-level temporal preference, not a recovered official score term.
// Its bounded cost breaks near ties without changing native velocities,
// rejecting an otherwise usable escape, or competing with a soft obstacle
// layer (whose smallest penalty is 0.6). A previous command is only supplied
// while it is being sent from a fresh, valid result in the same generation.
inline double continuity_cost(Velocity v,const Velocity* previous){
 if(!previous)return 0.;
 return std::min(.08,.10*std::abs(v.x-previous->x)+.16*std::abs(v.y-previous->y)+.16*std::abs(v.w-previous->w));
}
// Audited score kernel: b9f90..ba0cc. The outer NOS FSM is not claimed equivalent.
inline double score(const Primitive&p,const Pose& goal,const Penalties& hits,const Weights&w={}){
 double s=w.goal*std::hypot(goal.x-p.end.x,goal.y-p.end.y)+w.yaw*std::abs(wrap(std::atan2(goal.y,goal.x)-p.end.yaw))+w.lateral*std::abs(p.velocity.y);
 for(int l=0;l<3;++l)s+=w.obstacle[l]*hits.soft[l][p.id];
 if(goal.x>0&&p.velocity.x<=.05)s+=1.;
 return s;
}
inline Selection select(const Library&lib,const Pose&goal,const Penalties&hits,const Limits&lim,
                         const std::function<bool(const Primitive&)>& check,const Velocity* previous=nullptr,
                         Direction direction=Direction::Any,const double* route_heading=nullptr){
 Selection out;std::vector<std::pair<double,int>> ranked;ranked.reserve(count);
 for(const auto&p:lib.paths){auto v=p.velocity;
  if(std::abs(v.x)>lim.x+1e-6||std::abs(v.y)>lim.y+1e-6||std::abs(v.w)>lim.w+1e-6||std::hypot(v.x,v.y)>lim.planar+1e-6){++out.limit_rejected;continue;}
  if((direction==Direction::Forward&&v.x<-.001)||(direction==Direction::Reverse&&v.x>=-.05))continue;
  if(direction!=Direction::Any&&std::hypot(v.x,v.y)<.01&&std::abs(v.w)<.01)continue;
  if(hits.hard[p.id]>=1){++out.hard_rejected;continue;}
  double cost=score(p,goal,hits)+continuity_cost(v,previous);
  // Reverse travel keeps the body facing forward; looking at a rear goal must
  // not reward turning the body 180 degrees while backing.
  // Route following separates translation to the preview point from body yaw
  // along the current route segment. Short previews must not steer yaw toward
  // a lateral tracking error. Default native scoring remains unchanged.
  if(direction==Direction::Reverse||route_heading){
   double heading=direction==Direction::Reverse?goal.yaw:*route_heading;
   cost-=.5*std::abs(wrap(std::atan2(goal.y,goal.x)-p.end.yaw));cost+=(direction==Direction::Reverse?.5:1.)*std::abs(wrap(heading-p.end.yaw));
  }
  ranked.emplace_back(cost,p.id);
 }
 std::sort(ranked.begin(),ranked.end());
 // The binary scores all candidates, validates the best, marks it rejected and
 // retries up to 100 times. Sorting once preserves this ordering without rescans.
 for(auto [cost,id]:ranked){
  if(!check(lib.paths[id])){if(++out.terrain_rejected>100)break;continue;}
  out.id=id;out.score=cost;out.command=lib.paths[id].velocity;break;
 }return out;
}

enum class State {Idle,WaitingInputs,Tracking,Blocked,Arrived,Cancelled,Fault};
// Generation invalidation is shared by route replacement, cancellation, frame reset,
// sensor loss and gait errors. A worker result must match the whole generation.
struct Epoch {
 uint64_t value=0;State state=State::Idle;
 void invalidate(State s){++value;state=s;}
 bool accept(uint64_t token)const{return token==value&&state!=State::Cancelled&&state!=State::Fault;}
};
struct Gait {
 static constexpr int flat=0x3002,stairs=0x3003;
 int confirmed=0,pending=0;double requested_at=0;bool failed=false;
 // Acknowledgement of the request alone is not gait completion.
 void observe(int gait,bool fresh){if(fresh){confirmed=gait;if(gait==pending)pending=0;}}
 int request(int wanted,int motion_state,double now){
  if(failed||pending||confirmed==wanted||motion_state!=17)return 0;
  pending=wanted;requested_at=now;return wanted;
 }
 void tick(double now){if(pending&&now-requested_at>3.)failed=true;}
 bool may_enter_stairs()const{return !failed&&confirmed==stairs;}
};
}
