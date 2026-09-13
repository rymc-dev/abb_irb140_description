cmake_minimum_required(VERSION 3.8)
project(abb_irb140_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(CURL REQUIRED)

add_executable(pneumatic_gripper_controller src/pneumatic_gripper_controller.cpp)
target_include_directories(pneumatic_gripper_controller PRIVATE include)
ament_target_dependencies(pneumatic_gripper_controller rclcpp rclcpp_action sensor_msgs control_msgs)
target_link_libraries(pneumatic_gripper_controller CURL::libcurl)

install(TARGETS pneumatic_gripper_controller
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/irb140_pick_and_place_demo_real.py
  scripts/irb140_pick_and_place_demo_sim.py
  scripts/fix_depth_points_frame.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes launch config models worlds maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
