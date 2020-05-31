cmake_minimum_required(VERSION 3.0)
project(v4l2ucp VERSION 0.0.2)

# This doesn't work for mixed C/C++, v4l2ctrl.c is c, when that is removed can restore this
# add_compile_options(-std=c++1z)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")

find_package(catkin REQUIRED COMPONENTS
  ddynamic_reconfigure
  roscpp
  roslint
  std_msgs
)

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")
roslint_cpp()

set(V4L2UCP_VERSION 2.0.2)
add_definitions(-Wall -DV4L2UCP_VERSION="${V4L2UCP_VERSION}")

find_library(V4L2_LIBRARY v4l2)
if (V4L2_LIBRARY)
    MESSAGE(STATUS "Looking for libv4l - found")
else (V4L2_LIBRARY)
    MESSAGE(FATAL_ERROR "Looking for libv4l - not found.")
endif (V4L2_LIBRARY)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES v4l2ucp
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/..
)

add_executable(v4l2ucp
  src/v4l2controls.cpp
  src/v4l2ucp.cpp
)

target_link_libraries(v4l2ucp
  ${V4L2_LIBRARY}
  ${catkin_LIBRARIES}
)

add_executable(v4l2ctrl src/v4l2ctrl.c)
target_link_libraries(v4l2ctrl
  ${V4L2_LIBRARY}
  ${catkin_LIBRARIES}
)

install(TARGETS v4l2ucp v4l2ctrl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
