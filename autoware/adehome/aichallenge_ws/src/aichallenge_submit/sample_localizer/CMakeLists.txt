# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(sample_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(SAMPLE_LOCALIZER_LIB_SRC
  src/sample_localizer.cpp
)

set(SAMPLE_LOCALIZER_LIB_HEADERS
  include/sample_localizer/sample_localizer.hpp
  include/sample_localizer/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SAMPLE_LOCALIZER_LIB_SRC}
  ${SAMPLE_LOCALIZER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(SAMPLE_LOCALIZER_NODE_SRC
  src/sample_localizer_node.cpp
)

set(SAMPLE_LOCALIZER_NODE_HEADERS
  include/sample_localizer/sample_localizer_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${SAMPLE_LOCALIZER_NODE_SRC}
  ${SAMPLE_LOCALIZER_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::sample_localizer::SampleLocalizerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)
