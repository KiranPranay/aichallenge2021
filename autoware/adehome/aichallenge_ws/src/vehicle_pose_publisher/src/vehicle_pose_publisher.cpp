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

#include "vehicle_pose_publisher/vehicle_pose_publisher.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <iostream>

namespace autoware
{
namespace vehicle_pose_publisher
{

VehiclePosePublisher::VehiclePosePublisher(rclcpp::Node & node)
{
  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    "/lgsvl/gnss_odom",
    rclcpp::QoS{10},
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {on_odometry(*msg);});

  pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("/aichallenge/vehicle_pose", rclcpp::QoS{10});
}

void VehiclePosePublisher::on_odometry(const nav_msgs::msg::Odometry & msg)
{
  tf2::Transform map_base_link_transform;
  tf2::fromMsg(msg.pose.pose, map_base_link_transform);
  tf2::Quaternion fix_rotation(0, 0, 0.70710678, 0.70710678);
  map_base_link_transform.setRotation(fix_rotation * map_base_link_transform.getRotation());
  
  geometry_msgs::msg::PoseStamped vehicle_pose;
  vehicle_pose.header.frame_id = "map";
  vehicle_pose.header.stamp = msg.header.stamp;
  const auto pos = map_base_link_transform.getOrigin();
  const auto ori = map_base_link_transform.getRotation();
  vehicle_pose.pose.position.x = pos.x();
  vehicle_pose.pose.position.y = pos.y();
  vehicle_pose.pose.position.z = pos.z();
  vehicle_pose.pose.orientation.x = ori.x();
  vehicle_pose.pose.orientation.y = ori.y();
  vehicle_pose.pose.orientation.z = ori.z();
  vehicle_pose.pose.orientation.w = ori.w();

  pose_pub_->publish(vehicle_pose);
}

}
}  // namespace autoware
