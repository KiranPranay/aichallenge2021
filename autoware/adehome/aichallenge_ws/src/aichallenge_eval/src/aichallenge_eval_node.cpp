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

#include "aichallenge_eval/aichallenge_eval_node.hpp"

#include <lanelet2_core/geometry/LaneletMap.h>

#include <chrono>
#include <fstream>

using namespace std::chrono_literals;

namespace autoware
{
namespace aichallenge_eval
{

AichallengeEvalNode::AichallengeEvalNode(const rclcpp::NodeOptions & options)
:  Node("aichallenge_eval", options),
  verbose(true)
{
  rclcpp::Node& node = *this;

  time_sub_ = node.create_subscription<aichallenge_msgs::msg::TimeData>(
    "/aichallenge/time",
    rclcpp::QoS{10},
    [this](aichallenge_msgs::msg::TimeData::SharedPtr msg) {on_timedata(*msg);});
  pose_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aichallenge/vehicle_pose",
    rclcpp::QoS{10},
    [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {on_vehiclepose(*msg);});
  score_pub_ = node.create_publisher<aichallenge_msgs::msg::ScoreData>("/aichallenge/score", rclcpp::QoS{10});
  track_limit_penalty_pub_ = node.create_publisher<std_msgs::msg::Float32>("/aichallenge/track_limit_penalty", rclcpp::QoS{10});

  map_client_ = this->create_client<autoware_auto_msgs::srv::HADMapService>("/had_maps/HAD_Map_Service");

  request_osm_binary_map();
}

void AichallengeEvalNode::on_timedata(const aichallenge_msgs::msg::TimeData & msg)
{
  aichallenge_msgs::msg::ScoreData score;
  score.rawTime = msg.seconds;
  score.hasFinished = msg.hasFinished;
  score.contactPenalty = msg.contactPenalty;
  score.trackLimitPenalty = static_cast<float>(track_limit_penalty_);
  score.time = score.rawTime + score.contactPenalty + score.trackLimitPenalty;
  score_pub_->publish(score);

  std::ofstream ofs("/output/score.json");
  ofs << "{" << std::endl;
  ofs << "  \"time\": " << score.time << "," << std::endl;
  ofs << "  \"rawTime\": " << score.rawTime << "," << std::endl;
  ofs << "  \"hasFinished\": " << score.hasFinished << "," << std::endl;
  ofs << "  \"contactPenalty\": " << score.contactPenalty << "," << std::endl;
  ofs << "  \"trackLimitPenalty\": " << score.trackLimitPenalty << std::endl;
  ofs << "}" << std::endl;
  ofs.close();
}

void AichallengeEvalNode::on_vehiclepose(const geometry_msgs::msg::PoseStamped & msg)
{
  auto current_time = get_clock()->now();
  if (last_message_time_.nanoseconds() == 0) {
    last_message_time_ = current_time;
    return;
  }
  if (osm_map_ == nullptr) {
    return;
  }
  const auto primitives = lanelet::geometry::findWithin2d(osm_map_->laneletLayer, lanelet::BasicPoint2d{msg.pose.position.x, msg.pose.position.y}, 0.);
  const bool is_in_lane = !primitives.empty();
  if (!is_in_lane) {
    auto duration_secs = (current_time - last_message_time_).seconds();
    track_limit_penalty_ += duration_secs;
  }
  std_msgs::msg::Float32 track_limit_penalty_msg;
  track_limit_penalty_msg.data = static_cast<float>(track_limit_penalty_);
  track_limit_penalty_pub_->publish(track_limit_penalty_msg);
  last_message_time_ = current_time;
}

void AichallengeEvalNode::request_osm_binary_map()
{
  while (rclcpp::ok() && !map_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "HAD map service not available yet. Waiting...");
  }
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Client interrupted while waiting for map service to appear. Exiting.");
  }

  auto request = std::make_shared<autoware_auto_msgs::srv::HADMapService_Request>();
  request->requested_primitives.push_back(
    autoware_auto_msgs::srv::HADMapService_Request::FULL_MAP);

  auto result = map_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
    throw std::runtime_error("AichallengeEvalNode: Map service call fail");
  }

  // copy message to map
  autoware_auto_msgs::msg::HADMapBin msg = result.get()->map;

  // Convert binary map msg to lanelet2 map and set the map for global path planner
  osm_map_ = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(msg, osm_map_);
}

}  // namespace aichallenge_eval
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::aichallenge_eval::AichallengeEvalNode)
