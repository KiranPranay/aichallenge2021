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

#include "aichallenge_sample/aichallenge_sample_node.hpp"

#include <unistd.h>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

using namespace std::chrono_literals;

namespace autoware
{
namespace aichallenge_sample
{

AichallengeSampleNode::AichallengeSampleNode(const rclcpp::NodeOptions & options)
:  Node("aichallenge_sample", options),
  verbose(true),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_)
{
  rclcpp::Node& node = *this;

  clock_sub_ = node.create_subscription<rosgraph_msgs::msg::Clock>(
    "/lgsvl/clock",
    rclcpp::QoS{10},
    [this](rosgraph_msgs::msg::Clock::SharedPtr msg) {on_clock(*msg);});

  route_sub_ = node.create_subscription<autoware_auto_msgs::msg::Route>(
    "/planning/global_path",
    rclcpp::QoS{10},
    [this](autoware_auto_msgs::msg::Route::SharedPtr msg) {on_route(*msg);});

  goal_pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("/planning/goal_pose", rclcpp::QoS{10});
  timer_ = this->create_wall_timer(1s, [this](void) {on_timer();});
}

void AichallengeSampleNode::on_clock(const rosgraph_msgs::msg::Clock & msg)
{
  current_clock_ = msg;
}

void AichallengeSampleNode::on_route(const autoware_auto_msgs::msg::Route & msg)
{
  current_goal_pose_y_ = msg.goal_point.y;
}

void AichallengeSampleNode::on_timer()
{
  geometry_msgs::msg::TransformStamped base_link_tf;
  try {
    base_link_tf = tf2_buffer_.lookupTransform(
      "map", "base_link",
      rclcpp::Time(current_clock_.clock), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }

  auto is_vehicle_in_left_area = is_left_area(base_link_tf.transform.translation.y);
  auto is_goal_in_left_area = is_left_area(current_goal_pose_y_);

  if (is_vehicle_in_left_area && is_goal_in_left_area) {
    goal_pose_pub_->publish(right_goal_pose());
  }
  if (!is_vehicle_in_left_area && !is_goal_in_left_area) {
    goal_pose_pub_->publish(left_goal_pose());
  }
}

}  // namespace aichallenge_sample
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::aichallenge_sample::AichallengeSampleNode)
