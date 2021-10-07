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
/// \brief This file defines the aichallenge_sample_node class.

#ifndef AICHALLENGE_SAMPLE__AICHALLENGE_SAMPLE_NODE_HPP_
#define AICHALLENGE_SAMPLE__AICHALLENGE_SAMPLE_NODE_HPP_

#include <aichallenge_sample/aichallenge_sample.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <autoware_auto_msgs/msg/route.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

namespace autoware
{
namespace aichallenge_sample
{

/// \class AichallengeSampleNode
/// \brief ROS 2 Node for hello world.
class AICHALLENGE_SAMPLE_PUBLIC AichallengeSampleNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit AichallengeSampleNode(const rclcpp::NodeOptions & options);

  void on_timer();
  void on_clock(const rosgraph_msgs::msg::Clock & msg);
  void on_route(const autoware_auto_msgs::msg::Route & msg);

private:
  bool verbose;  ///< whether to use verbose output or not.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_{};
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_{};
  rclcpp::Subscription<autoware_auto_msgs::msg::Route>::SharedPtr route_sub_{};
  rclcpp::TimerBase::SharedPtr timer_{};
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  rosgraph_msgs::msg::Clock current_clock_{};
  double current_goal_pose_y_{};

  bool is_left_area(double y) const { return y > 400; }

  geometry_msgs::msg::PoseStamped left_goal_pose() const
  {
    auto goal_pose = geometry_msgs::msg::PoseStamped();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = 84.02432250976562;
    goal_pose.pose.position.y = 716.7021484375;
    goal_pose.pose.position.x = 0;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = -0.999993622826058;
    goal_pose.pose.orientation.w = 0.0035713172941581866;
    return goal_pose;
  }

  geometry_msgs::msg::PoseStamped right_goal_pose() const
  {
    auto goal_pose = geometry_msgs::msg::PoseStamped();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = -0.5061283111572266;
    goal_pose.pose.position.y = -1.4735573530197144;
    goal_pose.pose.position.x = 0;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = -0.006606096469884904;
    goal_pose.pose.orientation.w = 0.9999781795066484;
    return goal_pose;
  }

};
}  // namespace aichallenge_sample
}  // namespace autoware

#endif  // AICHALLENGE_SAMPLE__AICHALLENGE_SAMPLE_NODE_HPP_
