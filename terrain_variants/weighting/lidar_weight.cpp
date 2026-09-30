// Experiment-only adapter for the installed nvblox ABI. The callback itself
// remains the installed implementation; only its LiDAR integrator is configured.
#include <nvblox_ros/nvblox_node.hpp>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <stdexcept>

namespace nvblox {
bool NvbloxNode::processLidarPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) {
  using Original=bool (*)(NvbloxNode*,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr&);
  static const auto original=[] {
    dlerror();
    auto result=reinterpret_cast<Original>(dlsym(RTLD_NEXT,
      "_ZN6nvblox10NvbloxNode22processLidarPointcloudERKSt10shared_ptrIKN11sensor_msgs3msg12PointCloud2_ISaIvEEEE"));
    if(!result)throw std::runtime_error("LiDAR weighting: installed callback symbol missing");
    return result;
  }();
  const char* setting=std::getenv("SE2_LIDAR_TSDF_WEIGHTING");
  if(!setting)throw std::runtime_error("LiDAR weighting: explicit mode required");
  WeightingFunctionType target;
  if(std::strcmp(setting,"inverse_square_tsdf_distance_penalty")==0)
    target=WeightingFunctionType::kInverseSquareTsdfDistancePenalty;
  else if(std::strcmp(setting,"inverse_square")==0)
    target=WeightingFunctionType::kInverseSquareWeight;
  else throw std::runtime_error("LiDAR weighting: unsupported mode");
  auto& integrator=static_mapper_->lidar_tsdf_integrator();
  if(integrator.weighting_function_type()!=target)
    integrator.weighting_function_type(target);
  if(integrator.weighting_function_type()!=target)
    throw std::runtime_error("LiDAR weighting: setting did not take effect");
  RCLCPP_INFO_ONCE(get_logger(),"SE2_EFFECTIVE_LIDAR_WEIGHTING=%s; installed callback retained",
      to_string(integrator.weighting_function_type()).c_str());
  return original(this,cloud);
}
}  // namespace nvblox
