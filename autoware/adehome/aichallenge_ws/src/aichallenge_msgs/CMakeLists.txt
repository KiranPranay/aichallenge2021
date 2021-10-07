# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(aichallenge_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeData.idl"
  "msg/ScoreData.idl"
  DEPENDENCIES
    "builtin_interfaces"
    "std_msgs"
  ADD_LINTER_TESTS
)

ament_auto_package()
