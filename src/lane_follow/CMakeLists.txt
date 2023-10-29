cmake_minimum_required(VERSION 3.8)
project(lane_follow)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lane_follow.cpp
#   src/lane_follow_node.cpp
#   DIRECTORY src
)

ament_auto_add_executable(${PROJECT_NAME}_node src/lane_follow_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
