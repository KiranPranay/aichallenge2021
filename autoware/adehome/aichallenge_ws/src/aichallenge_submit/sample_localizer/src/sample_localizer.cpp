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

#include "sample_localizer/sample_localizer.hpp"
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/utils.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <iostream>

namespace autoware
{
namespace sample_localizer
{

SampleLocalizer::SampleLocalizer(rclcpp::Node & node)
: node_(&node),
  tf2_buffer_(node.get_clock()),
  tf2_listener_(tf2_buffer_)
{
  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);

  pose_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aichallenge/vehicle_pose",
    rclcpp::QoS{10},
    [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {on_pose(*msg);});

  tf_pub_ = node.create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS{10});
}

void SampleLocalizer::on_pose(const geometry_msgs::msg::PoseStamped & msg)
{
  tf2::Transform map_base_link_transform;
  tf2::fromMsg(msg.pose, map_base_link_transform);

  geometry_msgs::msg::TransformStamped odom_tf;
  try {
    odom_tf = tf2_buffer_.lookupTransform(
      "odom", "base_link",
      rclcpp::Time(msg.header.stamp), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    return;
  }
  tf2::Quaternion odom_rotation{odom_tf.transform.rotation.x,
    odom_tf.transform.rotation.y, odom_tf.transform.rotation.z, odom_tf.transform.rotation.w};
  tf2::Vector3 odom_translation{odom_tf.transform.translation.x, odom_tf.transform.translation.y,
    odom_tf.transform.translation.z};
  const tf2::Transform odom_base_link_transform{odom_rotation, odom_translation};

  const auto map_odom_tf = map_base_link_transform * odom_base_link_transform.inverse();
  geometry_msgs::msg::TransformStamped map_odom_tf_msg;
  map_odom_tf_msg.transform = tf2::toMsg(map_odom_tf);
  map_odom_tf_msg.header = msg.header;
  map_odom_tf_msg.header.frame_id = "map";
  map_odom_tf_msg.child_frame_id = "odom";

  tf2_msgs::msg::TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(map_odom_tf_msg));

  tf_pub_->publish(tf_msg);
}

}
}  // namespace autoware
