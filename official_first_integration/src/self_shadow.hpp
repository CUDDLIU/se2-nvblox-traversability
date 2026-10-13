#pragma once
#include "world.hpp"
namespace first {
// M20 central chassis collision box: .75 x .09 x .14 m, centered at
// base_link (robot_model/upstream/M20/urdf/M20.urdf). The AIRY aperture faces
// forward/rearward, not through the underside of this box. Only reject rays
// crossing the strict interior of its lower face; do not expand a range blind
// zone or the leg/footprint envelope.
// AIRY's decoded endpoint includes axial RZ=.04532 and rotating radial
// hypot(RX=.0075, RY=.00664). Undo that offset for ray geometry only; endpoints
// and public TF remain unchanged. The calibrated azimuth correction is small;
// retain grazing rays within .5 mm of the box boundary.
inline Point airy_optical_origin(Point p,int ring){
 double radial=std::hypot(p.y,p.z+.013),rxy=std::hypot(.0075,.00664);
 double s=radial>1e-9?rxy/radial:0.;
 return {(ring<96?1.:-1.)*(.32028+.04532),s*p.y,-.013+s*(p.z+.013)};
}
inline bool chassis_bottom_shadow(Point p,int ring){
 if(ring<0||ring>=192||!std::isfinite(p.x+p.y+p.z)||p.z>=-.0705)return false;
 auto o=airy_optical_origin(p,ring);
 double u=(-.07-o.z)/(p.z-o.z);
 if(!(u>0&&u<1))return false;
 double x=o.x+u*(p.x-o.x),y=o.y+u*(p.y-o.y);
 return std::abs(x)<.3745&&std::abs(y)<.0445;
}
}
