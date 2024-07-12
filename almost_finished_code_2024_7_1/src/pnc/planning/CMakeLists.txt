cmake_minimum_required(VERSION 2.8.3)

project(planning)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  mapping
  vehicle_params
  car_msgs
  decomp_ros_utils
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(plan_node src/plan_test_node.cpp src/plan_manager.cpp src/kino_astar.cpp src/traj_optimizer.cpp)

target_link_libraries(plan_node
  ${catkin_LIBRARIES}
  ompl
)
