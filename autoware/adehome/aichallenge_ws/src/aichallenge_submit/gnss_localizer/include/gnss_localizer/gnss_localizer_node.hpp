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
/// \brief This file defines the gnss_localizer_node class.

#ifndef GNSS_LOCALIZER__GNSS_LOCALIZER_NODE_HPP_
#define GNSS_LOCALIZER__GNSS_LOCALIZER_NODE_HPP_

#include <gnss_localizer/gnss_localizer.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

namespace autoware
{
namespace gnss_localizer
{

/// \class GnssLocalizerNode
/// \brief ROS 2 Node for hello world.
class GNSS_LOCALIZER_PUBLIC GnssLocalizerNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit GnssLocalizerNode(const rclcpp::NodeOptions & options);

  /// \brief print hello
  /// return 0 if successful.
  int32_t print_hello() const;

private:
  bool verbose;  ///< whether to use verbose output or not.
  void on_odometry(const nav_msgs::msg::Odometry & msg);
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
};
}  // namespace gnss_localizer
}  // namespace autoware

#endif  // GNSS_LOCALIZER__GNSS_LOCALIZER_NODE_HPP_
