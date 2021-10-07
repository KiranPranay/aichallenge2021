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
/// \brief This file defines the sample_localizer_node class.

#ifndef SAMPLE_LOCALIZER__SAMPLE_LOCALIZER_NODE_HPP_
#define SAMPLE_LOCALIZER__SAMPLE_LOCALIZER_NODE_HPP_

#include <sample_localizer/sample_localizer.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace sample_localizer
{

/// \class SampleLocalizerNode
class SAMPLE_LOCALIZER_PUBLIC SampleLocalizerNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit SampleLocalizerNode(const rclcpp::NodeOptions & options);

private:
  bool verbose;  ///< whether to use verbose output or not.
  std::shared_ptr<SampleLocalizer> sample_localizer;
};
}  // namespace sample_localizer
}  // namespace autoware

#endif  // SAMPLE_LOCALIZER__SAMPLE_LOCALIZER_NODE_HPP_
