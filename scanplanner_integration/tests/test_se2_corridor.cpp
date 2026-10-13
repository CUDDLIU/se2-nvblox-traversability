#include <plan_env/se2_corridor.h>
#include <cassert>
#include <iostream>
#include <random>
int main(){
  scan_reference::Corridor c;
  std::vector<double> data={1,123,40,.2,3,
    .6,1,0,4, 0,0,1,0,1,1,0,1,
    .6,1024,0,4, 2,0,3,0,3,1,2,1,
    3.6,1,0,4, 0,0,1,0,1,1,0,1};
  assert(c.decode(data));Eigen::Vector2d d;
  assert(c.distance({.5,.5,.6},0,d)&&d.norm()<1e-12);
  assert(c.distance({1.2,.5,.6},0,d)&&std::abs(d.x()-.2)<1e-12);
  assert(c.distance({1.2,.5,.6},M_PI/2,d)&&std::abs(d.x()+.8)<1e-12);
  assert(!c.distance({.5,.5,1.6},0,d));
  assert(c.distance({.5,.5,3.6},0,d)&&d.norm()<1e-12);
  auto malformed=data;malformed.pop_back();assert(!c.decode(malformed)&&c.stamp==123);
  malformed=data;malformed[2]=65;assert(!c.decode(malformed));
  malformed=data;malformed[6]=std::numeric_limits<double>::quiet_NaN();assert(!c.decode(malformed));
  scan_reference::Geometry ref;ref.poses={{0,0,.6,0},{3,0,.6,0}};
  Eigen::MatrixXd p(3,8);p.row(0).setLinSpaced(8,1.1,1.8);p.row(1).setConstant(.5);p.row(2).setConstant(.6);
  Eigen::MatrixXd gradient;c.curveCost(p,ref,gradient);
  for(int i=0;i<p.cols();++i){Eigen::MatrixXd q=p,dummy;q(0,i)+=1e-6;
    double a=c.curveCost(q,ref,dummy);q(0,i)-=2e-6;double b=c.curveCost(q,ref,dummy);
    assert(std::abs((a-b)/2e-6-gradient(0,i))<1e-6);
  }
  // Measured body-height smoothing must not select another ground layer.
  scan_reference::Corridor layer;
  assert(layer.decode({1,124,40,.01,2,
      .6,1,0,4,0,0,1,0,1,1,0,1,
      .63,1,0,4,1,0,2,0,2,1,1,1}));
  p.row(0).setConstant(1.5);p.row(2).setConstant(.63);
  assert(layer.curveCost(p,ref,gradient)>.1);
  // Spatial index must preserve exhaustive nearest-region distances, masks,
  // and layers, including searches outside the region bounds.
  std::vector<double> many={1,125,40,.01,200};
  for(int i=0;i<200;++i){double x=(i%20)*.1,y=(i/20)*.1;
    many.insert(many.end(),{.6+(i%3)*.01,double(1u<<(i%20)),0.,4.,x,y,x+.09,y,x+.09,y+.09,x,y+.09});}
  assert(layer.decode(many));std::mt19937 rng(42);std::uniform_real_distribution<double> xy(-.5,2.5);
  for(int i=0;i<1000;++i){Eigen::Vector3d point(xy(rng),xy(rng),.6+(i%5)*.01);int bin=i%40;
    double best=std::numeric_limits<double>::infinity();
    for(const auto&r:layer.regions)if((r.mask&(uint64_t(1)<<bin))&&std::abs(r.z-point.z())<=layer.max_step)
      best=std::min(best,r.displacement(point.head<2>()).squaredNorm());
    bool found=layer.distance(point,bin*2*M_PI/40,d);assert(found==std::isfinite(best));
    if(found)assert(std::abs(d.squaredNorm()-best)<1e-12);
  }
  // A pose witness attracts the curve to a point, not its entire parent cell.
  assert(c.decode({1,124,40,.2,1,.6,1,0,1,.5,.5}));
  assert(c.distance({.6,.5,.6},0,d)&&std::abs(d.x()-.1)<1e-12);
  std::cout<<"SE2 corridor yaw/layer/parser/gradient: passed\n";
}
