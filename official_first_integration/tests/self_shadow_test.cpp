#include "self_shadow.hpp"
#include <iostream>
#define REQUIRE(v) do{if(!(v))throw std::runtime_error(#v);}while(0)
int main(){
 // Real captured short returns (front and rear), lying beyond the opaque
 // underside rather than inside the URDF endpoint crop.
 REQUIRE(first::chassis_bottom_shadow({.418306,-.030646,-.347374},10));
 REQUIRE(first::chassis_bottom_shadow({.425629,-.039077,-.360559},11));
 REQUIRE(first::chassis_bottom_shadow({-.404515,.054800,-.320677},104));
 REQUIRE(first::chassis_bottom_shadow({-.406429,-.091910,-.327231},104));
 // Visible close objects, side legs, a vertical narrow pole and stair fronts
 // remain visible; intensity and range alone never determine rejection.
 REQUIRE(!first::chassis_bottom_shadow({.45,0,-.2},10));
 REQUIRE(!first::chassis_bottom_shadow({.60,0,-.4},10));
 REQUIRE(!first::chassis_bottom_shadow({.43,.25,-.3},10));
 REQUIRE(!first::chassis_bottom_shadow({.42,0,.1},10));
 REQUIRE(!first::chassis_bottom_shadow({.50,0,-.5},10));
 for(double z=-.45;z<=.1;z+=.01)REQUIRE(!first::chassis_bottom_shadow({.55,.05,z},20));
 for(double z=-.45;z<=-.15;z+=.01)REQUIRE(!first::chassis_bottom_shadow({.65,0,z},20));
 // Same raw ray geometry for both physical sensors.
 for(double x=.38;x<.8;x+=.02)for(double z=-.7;z<0;z+=.02)
  REQUIRE(first::chassis_bottom_shadow({x,.02,z},10)==first::chassis_bottom_shadow({-x,.02,z},106));
 REQUIRE(!first::chassis_bottom_shadow({.4,0,-.3},192));
 std::cout<<"PASS chassis-bottom occlusion, captured front/rear echoes, visible close pole and step, sensor symmetry\n";
}
