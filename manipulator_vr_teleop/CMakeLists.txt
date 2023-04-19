cmake_minimum_required(VERSION 3.8)
project(manipulator_vr_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(manipulator_vr_teleop_interface REQUIRED)
find_package(ur_msgs)
find_package(std_srvs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(eff_cmd src/eff_cmd.cpp)
target_include_directories(eff_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(eff_cmd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  eff_cmd
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(occulus_cmd src/occulus_cmd.cpp)
target_include_directories(occulus_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(occulus_cmd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  occulus_cmd
  "moveit_ros_planning_interface"
  "manipulator_vr_teleop_interface"
  "rclcpp"
)

add_executable(occulus_direction src/occulus_direction.cpp)
target_include_directories(occulus_direction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(occulus_direction PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  occulus_direction
  "moveit_ros_planning_interface"
  "manipulator_vr_teleop_interface"
  "rclcpp"
)

add_executable(gripper_cmd src/gripper_cmd.cpp)
target_include_directories(gripper_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gripper_cmd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  gripper_cmd
  "rclcpp"
  "ur_msgs"
  "std_srvs"
)

install(TARGETS
  eff_cmd
  occulus_cmd
  occulus_direction
  gripper_cmd
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
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
