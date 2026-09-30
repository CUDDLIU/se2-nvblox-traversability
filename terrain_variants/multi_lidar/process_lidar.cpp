// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Derived from isaac_ros_nvblox release-3.2 processLidarPointcloud.
// Experiment-only ABI interposition; no installed file or public TF change.
#include <nvblox_ros/nvblox_node.hpp>
#include <nvblox/utils/delays.h>
#include <nvblox/utils/rates.h>
#include "split_cloud.hpp"
#include <cstdlib>

namespace nvblox {
bool NvbloxNode::processLidarPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) {
  timing::Timer total("ros/lidar");
  timing::Rates::tick("ros/lidar");
  const rclcpp::Time stamp=cloud->header.stamp;
  if(!shouldProcess(stamp,integrate_lidar_last_time_,params_.integrate_lidar_rate_hz) ||
     stamp<=shape_clearing_last_time_) return true;
  timing::Timer transform_timer("ros/lidar/transform");
  Transform world_body;
  if(!transformer_.lookupTransformToGlobalFrame(cloud->header.frame_id,
      cloud->header.stamp,&world_body))return false;
  transform_timer.Stop();
  timing::Timer split_timer("ros/lidar/split");
  // Bad layout/frame/ring aborts this isolated experiment rather than silently
  // accepting a cloud under an unverified physical-sensor assumption.
  const char* setting=std::getenv("SE2_LIDAR_NATIVE_AXES");
  const bool native_axes=setting && std::strcmp(setting,"1")==0;
  setting=std::getenv("SE2_LIDAR_OPTICAL_ORIGIN");
  const bool optical=setting && std::strcmp(setting,"1")==0;
  const auto parts=se2_multi_lidar::split(*cloud,native_axes,optical);
  split_timer.Stop();
  Lidar lidar=(params_.use_non_equal_vertical_fov_lidar_params)?
    Lidar(params_.lidar_width,params_.lidar_height,params_.lidar_min_valid_range_m,
      params_.lidar_max_valid_range_m,params_.min_angle_below_zero_elevation_rad,
      params_.max_angle_above_zero_elevation_rad):
    Lidar(params_.lidar_width,params_.lidar_height,params_.lidar_min_valid_range_m,
      params_.lidar_max_valid_range_m,params_.lidar_vertical_fov_rad);
  RCLCPP_INFO_ONCE(get_logger(),"MULTI_LIDAR v1: ring 0..95 front,96..191 rear; "
      "physical origins +/-0.32028,0,-0.013 m; one timestamp, two integrations");
  RCLCPP_INFO_ONCE(get_logger(),"MULTI_LIDAR native_axes=%d projection=%d x %d",
      native_axes,static_cast<int>(params_.lidar_width),static_cast<int>(params_.lidar_height));
  RCLCPP_INFO_ONCE(get_logger(),"MULTI_LIDAR optical_plane=%d origin_x=+/-%.5f; rotating radial origin still approximated",
      optical,se2_multi_lidar::originX(0,optical));
  timing::Timer pair_timer("ros/lidar/integration");
  for(int sensor=0;sensor<2;++sensor){
    if(parts[sensor]->width==0)continue;
    Transform body_sensor=Transform::Identity();
    body_sensor.translation()=Vector3f(se2_multi_lidar::originX(sensor,optical),0.f,
                                     se2_multi_lidar::kOriginZ);
    if(native_axes){
      // Existing recorded front Rz(-pi)Ry(-pi/2), rear Ry(-pi/2).
      // Exact axis permutations avoid artificial floating point seam offsets.
      if(sensor==0)body_sensor.linear()<<0,0,1,0,-1,0,1,0,0;
      else body_sensor.linear()<<0,0,-1,0,1,0,1,0,0;
    }
    timing::Timer conversion("ros/lidar/conversion");
    // The generic spherical projection rejects invalid/out-of-FoV points. Its
    // sparse image is expected; no vendor scan-ring grid assumptions are used.
    pointcloud_converter_.depthImageFromPointcloudGPU(parts[sensor],lidar,&pointcloud_image_);
    conversion.Stop();
    timing::Timer integrate(sensor==0?"ros/lidar/front_integrated":"ros/lidar/rear_integrated");
    static_mapper_->integrateLidarDepth(pointcloud_image_,world_body*body_sensor,lidar);
  }
  pair_timer.Stop();
  timing::Delays::tick("ros/pointcloud_integration",nvblox::Time(stamp.nanoseconds()),
                       nvblox::Time(now().nanoseconds()));
  newest_integrated_depth_time_=std::max(stamp,newest_integrated_depth_time_);
  integrate_lidar_last_time_=stamp;
  return true;
}
}  // namespace nvblox
