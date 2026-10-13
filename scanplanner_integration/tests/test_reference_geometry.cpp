#include <plan_env/reference_geometry.h>
#include <bspline_opt/route_curve_cost.h>
#include <cassert>
#include <iostream>
int main(){
  scan_reference::Geometry geometry;
  assert(geometry.yawAt({1,2,3},.7)==.7);
  // Sideways translation must preserve the body's independently planned yaw.
  geometry.poses={{0,0,.6,0},{0,2,.6,0}};
  assert(std::abs(geometry.yawAt({0,1,.6},M_PI/2))<1e-12);
  geometry.poses={{0,0,.6,3.1},{1,0,.6,-3.1}};
  assert(std::abs(std::abs(geometry.yawAt({.5,0,.6},0))-M_PI)<1e-12);
  geometry.poses={{0,0,.6,0},{1,0,.6,0},{1,0,3.6,1},{0,0,3.6,1}};
  assert(std::abs(geometry.yawAt({.4,0,3.6},0)-1)<1e-12);
  Eigen::MatrixXd p(3,8);p<<0,.1,.2,.4,.6,.8,.9,1,
    .03,.04,.08,.1,.08,.07,.05,.03,
    .6,.6,.6,.6,.6,.6,.6,.6;
  std::vector<Eigen::Vector3d> route{{0,0,.6},{1,0,.6}};
  Eigen::MatrixXd grad;
  double cost=scan_reference::curveCost(p,route,grad);assert(cost>0);
  for(int c=0;c<p.cols();++c)for(int r=0;r<3;++r){
    Eigen::MatrixXd q=p,dummy;q(r,c)+=1e-6;
    const double plus=scan_reference::curveCost(q,route,dummy);q(r,c)-=2e-6;
    const double minus=scan_reference::curveCost(q,route,dummy);
    assert(std::abs((plus-minus)/2e-6-grad(r,c))<1e-7);
  }
  p.row(1).setZero();assert(scan_reference::curveCost(p,route,grad)<1e-20);
  assert(scan_reference::curveCost(p,{},grad)==0 && grad.norm()==0);
  std::cout << "reference body yaw and cubic-curve gradient: passed\n";
}
