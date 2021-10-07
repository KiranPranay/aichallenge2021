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
/// \brief This file defines the aichallenge_eval_node class.

#ifndef AICHALLENGE_EVAL__AICHALLENGE_EVAL_NODE_HPP_
#define AICHALLENGE_EVAL__AICHALLENGE_EVAL_NODE_HPP_

#include <aichallenge_eval/aichallenge_eval.hpp>
#include <aichallenge_msgs/msg/time_data.hpp>
#include <aichallenge_msgs/msg/score_data.hpp>

#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <std_msgs/msg/float32.hpp>
#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_msgs/msg/route.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <had_map_utils/had_map_conversion.hpp>
#include <common/types.hpp>

namespace autoware
{
namespace aichallenge_eval
{

/// \class AichallengeEvalNode
/// \brief ROS 2 Node for hello world.
class AICHALLENGE_EVAL_PUBLIC AichallengeEvalNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit AichallengeEvalNode(const rclcpp::NodeOptions & options);

private:
  void on_timedata(const aichallenge_msgs::msg::TimeData & msg);
  void on_vehiclepose(const geometry_msgs::msg::PoseStamped & msg);
  void request_osm_binary_map();

  bool verbose;  ///< whether to use verbose output or not.

  rclcpp::Publisher<aichallenge_msgs::msg::ScoreData>::SharedPtr score_pub_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr track_limit_penalty_pub_{};
  rclcpp::Subscription<aichallenge_msgs::msg::TimeData>::SharedPtr time_sub_{};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_{};
  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr map_client_;
  std::shared_ptr<lanelet::LaneletMap> osm_map_;
  double track_limit_penalty_;
  rclcpp::Time last_message_time_;

};
}  // namespace aichallenge_eval
}  // namespace autoware

#endif  // AICHALLENGE_EVAL__AICHALLENGE_EVAL_NODE_HPP_
