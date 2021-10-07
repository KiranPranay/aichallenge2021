cmake_minimum_required(VERSION 3.5)

project(aichallenge_launch)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install stuff
ament_auto_package(INSTALL_TO_SHARE
  data
  launch
  param
)
