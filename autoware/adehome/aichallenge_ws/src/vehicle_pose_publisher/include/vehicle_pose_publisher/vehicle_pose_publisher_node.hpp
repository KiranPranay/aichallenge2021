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
/// \brief This file defines the vehicle_pose_publisher_node class.

#ifndef VEHICLE_POSE_PUBLISHER__VEHICLE_POSE_PUBLISHER_NODE_HPP_
#define VEHICLE_POSE_PUBLISHER__VEHICLE_POSE_PUBLISHER_NODE_HPP_

#include <vehicle_pose_publisher/vehicle_pose_publisher.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace vehicle_pose_publisher
{

/// \class VehiclePosePublisherNode
class VEHICLE_POSE_PUBLISHER_PUBLIC VehiclePosePublisherNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit VehiclePosePublisherNode(const rclcpp::NodeOptions & options);

private:
  bool verbose;  ///< whether to use verbose output or not.
  std::shared_ptr<VehiclePosePublisher> vehicle_pose_publisher;
};
}  // namespace vehicle_pose_publisher
}  // namespace autoware

#endif  // VEHICLE_POSE_PUBLISHER__VEHICLE_POSE_PUBLISHER_NODE_HPP_
