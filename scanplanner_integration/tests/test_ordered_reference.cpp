#include <plan_manage/ordered_reference.h>
#include <cassert>
#include <iostream>
using Eigen::Vector3d;
using scan_planner::remainingReference;
int main() {
  const std::vector<Vector3d> corner{{0,0,0},{1,0,0},{1,1,0}};
  auto p=remainingReference({.3,.1,0},corner);
  assert(p.size()==3 && p[1].isApprox(corner[1]) && p[2].isApprox(corner[2]));
  p=remainingReference({-.3,.1,0},corner);
  assert(p.size()==4 && p[1].isApprox(corner[0]));
  p=remainingReference({1.01,.02,0},corner);
  assert(p.size()==2 && p[1].isApprox(corner[2]));
  // A nearby future branch must not be mistaken for current progress.
  const std::vector<Vector3d> fold{{0,0,0},{1,0,0},{1,.1,0},{0,.1,0}};
  p=remainingReference({.1,.09,0},fold);
  assert(p.size()==4 && p[1].isApprox(fold[1]));
  // Horizontal coincidence is not arrival at a landing on another floor.
  const std::vector<Vector3d> levels{{0,0,0},{1,0,1},{1,1,1}};
  p=remainingReference({1,0,0},levels);
  assert(p.size()==3 && p[1].isApprox(levels[1]));
  // Every replan uses measured progress even when the reference is unchanged.
  for(double x=.1;x<.9;x+=.1) {
    p=remainingReference({x,0,0},corner);
    assert(p[1].x()>x && p.back().isApprox(corner.back()));
  }
  std::cout << "ordered reference splice: passed\n";
}
