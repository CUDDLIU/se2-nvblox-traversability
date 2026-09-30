#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace se2_deskew {
struct Pose {double stamp; Eigen::Vector3d p; Eigen::Quaterniond q;};
inline Pose interpolate(const std::deque<Pose>& poses,double t) {
  if(poses.empty() || t<poses.front().stamp || t>poses.back().stamp)
    throw std::out_of_range("pose does not bracket point time");
  auto b=std::lower_bound(poses.begin(),poses.end(),t,[](const Pose& p,double s){return p.stamp<s;});
  if(b==poses.begin() || b->stamp==t)return *b;
  const auto& a=*(b-1);double u=(t-a.stamp)/(b->stamp-a.stamp);
  return {t,a.p+u*(b->p-a.p),a.q.slerp(u,b->q).normalized()};
}
inline Eigen::Isometry3d relative(const Pose& reference,const Pose& point) {
  Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
  out.linear()=reference.q.conjugate().toRotationMatrix()*point.q.toRotationMatrix();
  out.translation()=reference.q.conjugate()*(point.p-reference.p);
  return out;
}
}
