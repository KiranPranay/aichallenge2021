// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the vehicle_pose_publisher class.

#ifndef VEHICLE_POSE_PUBLISHER__VEHICLE_POSE_PUBLISHER_HPP_
#define VEHICLE_POSE_PUBLISHER__VEHICLE_POSE_PUBLISHER_HPP_

#include <vehicle_pose_publisher/visibility_control.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>

namespace autoware
{
namespace vehicle_pose_publisher
{

class VEHICLE_POSE_PUBLISHER_PUBLIC VehiclePosePublisher
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit VehiclePosePublisher(rclcpp::Node & node);

private:
  void on_odometry(const nav_msgs::msg::Odometry & msg);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{};

};

}  // namespace vehicle_pose_publisher
}  // namespace autoware

#endif  // VEHICLE_POSE_PUBLISHER__VEHICLE_POSE_PUBLISHER_HPP_
