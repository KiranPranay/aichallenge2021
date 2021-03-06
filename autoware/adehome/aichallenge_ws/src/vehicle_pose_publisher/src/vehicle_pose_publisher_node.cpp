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

#include "vehicle_pose_publisher/vehicle_pose_publisher_node.hpp"

namespace autoware
{
namespace vehicle_pose_publisher
{

VehiclePosePublisherNode::VehiclePosePublisherNode(const rclcpp::NodeOptions & options)
:  Node("vehicle_pose_publisher", options),
  verbose(true)
{
  this->vehicle_pose_publisher = std::make_unique<VehiclePosePublisher>(*this);
}

}  // namespace vehicle_pose_publisher
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::vehicle_pose_publisher::VehiclePosePublisherNode)
