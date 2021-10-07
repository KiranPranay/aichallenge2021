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

#include "sample_localizer/sample_localizer_node.hpp"

namespace autoware
{
namespace sample_localizer
{

SampleLocalizerNode::SampleLocalizerNode(const rclcpp::NodeOptions & options)
:  Node("sample_localizer", options),
  verbose(true)
{
  this->sample_localizer = std::make_unique<SampleLocalizer>(*this);
}

}  // namespace sample_localizer
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::sample_localizer::SampleLocalizerNode)
