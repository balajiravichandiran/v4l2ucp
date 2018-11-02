cmake_minimum_required(VERSION 3.5)
project(v4l2ucp)

set(V4L2UCP_VERSION 2.0.2)
add_definitions(-Wall -DV4L2UCP_VERSION="${V4L2UCP_VERSION}")

find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rosidl_typesupport_cpp REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Control.msg"
  "srv/SetControl.srv"
)

find_library(V4L2_LIBRARY v4l2)
if (V4L2_LIBRARY)
    MESSAGE(STATUS "Looking for libv4l - found")
else (V4L2_LIBRARY)
    MESSAGE(FATAL_ERROR "Looking for libv4l - not found.")
endif (V4L2_LIBRARY)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/..
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/v4l2controls.cpp
  src/v4l2ucp.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${V4L2_LIBRARY}
)

ament_target_dependencies(${PROJECT_NAME}_node
  "builtin_interfaces"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# TODO(lucasw) is this application needed?  It is provided by the 'real' (non-ros) v4l2ucp
add_executable(v4l2ctrl src/v4l2ctrl.c)
target_link_libraries(v4l2ctrl
  ${V4L2_LIBRARY}
)

install(TARGETS ${PROJECT_NAME}_node v4l2ctrl DESTINATION lib/${PROJECT_NAME})

ament_package()
