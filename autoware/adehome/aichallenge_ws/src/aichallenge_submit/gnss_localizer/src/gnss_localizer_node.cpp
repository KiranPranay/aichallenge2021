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

#include "gnss_localizer/gnss_localizer_node.hpp"

#include <unistd.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_msgs/msg/tf_message.hpp>

namespace autoware
{
namespace gnss_localizer
{

GnssLocalizerNode::GnssLocalizerNode(const rclcpp::NodeOptions & options)
:  Node("gnss_localizer", options),
  verbose(true),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_)
{
  rclcpp::Node& node = *this;

  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);

  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    "/lgsvl/gnss_odom",
    rclcpp::QoS{10},
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {on_odometry(*msg);});

  tf_pub_ = node.create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS{10});

  sleep(1);
}

void GnssLocalizerNode::on_odometry(const nav_msgs::msg::Odometry & msg)
{
  tf2::Transform map_base_link_transform;
  tf2::fromMsg(msg.pose.pose, map_base_link_transform);
  tf2::Quaternion fix_rotation(0, 0, 0.70710678, 0.70710678);
  map_base_link_transform.setRotation(fix_rotation * map_base_link_transform.getRotation());

  geometry_msgs::msg::TransformStamped odom_tf;
  try {
    odom_tf = tf2_buffer_.lookupTransform(
      "odom", "base_link",
      rclcpp::Time(msg.header.stamp), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
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

int32_t GnssLocalizerNode::print_hello() const
{
  return gnss_localizer::print_hello();
}

}  // namespace gnss_localizer
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_localizer::GnssLocalizerNode)
