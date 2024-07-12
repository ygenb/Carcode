cmake_minimum_required(VERSION 3.0.2)
project(mapping)


add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  car_msgs
  roscpp
  cv_bridge
  tf
  pcl_ros
  pcl_conversions
  pose_utils
  car_msgs
  vehicle_params
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS roscpp tf pcl_ros pcl_conversions cv_bridge car_msgs
  LIBRARIES ${PROJECT_NAME} map_manager
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# local map using 3d circular buffer
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# mapping 
add_library(map_manager
  src/map_manager.cpp
  src/raycast.cpp
)

target_link_libraries(map_manager
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# add_dependencies(map_manager
#   ${catkin_EXPORTED_TARGETS}
# )

# mapping visualization node
add_executable(${PROJECT_NAME}_vis_node
  src/${PROJECT_NAME}_vis_node.cpp)

target_link_libraries(${PROJECT_NAME}_vis_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
# add_dependencies(${PROJECT_NAME}_vis_node
#   ${catkin_EXPORTED_TARGETS}
# )
