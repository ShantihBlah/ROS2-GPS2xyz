cmake_minimum_required(VERSION 3.5)
project(geodetic_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)


# declare the message files to generate code for
set(msg_files
   "msg/Initgps.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   DEPENDENCIES builtin_interfaces
)



ament_export_dependencies(rosidl_default_runtime)

###################################################################################
#build
##################################################################################
include_directories(
	include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(gps_to_pose_conversion_node src/gps_to_pose_conversion_node.cpp)
ament_target_dependencies(gps_to_pose_conversion_node nav_msgs rclcpp tf2 tf2_ros std_msgs sensor_msgs std_srvs geometry_msgs)
rosidl_target_interfaces(gps_to_pose_conversion_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(set_gps_reference_node src/set_gps_reference_node.cpp)
ament_target_dependencies(set_gps_reference_node rclcpp std_msgs sensor_msgs std_srvs example_interfaces)
rosidl_target_interfaces(set_gps_reference_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

#add_executable(subcription_test src/subcription_test.cpp)
#ament_target_dependencies(subcription_test rclcpp std_msgs sensor_msgs std_srvs geometry_msgs)
#rosidl_target_interfaces(subcription_test ${PROJECT_NAME} "rosidl_typesupport_cpp")

#add_executable(set_gps_node_2 src/set_gps_node_2.cpp)
#ament_target_dependencies(set_gps_node_2 rclcpp std_msgs sensor_msgs std_srvs example_interfaces)

#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

#add_executable(set_and_get_param src/set_and_get_param.cpp)
#ament_target_dependencies(set_and_get_param rclcpp )

#add_executable(listener src/listener.cpp)
#ament_target_dependencies(listener rclcpp std_msgs rclcpp_components)

#add_executable(talker src/talker.cpp)
#ament_target_dependencies(talker rclcpp std_msgs sensor_msgs rclcpp_components)


################################################################################
# Install
################################################################################


install(TARGETS
	set_gps_reference_node
  gps_to_pose_conversion_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})
