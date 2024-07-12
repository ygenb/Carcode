cmake_minimum_required(VERSION 3.0.2)
project(vehicle_params)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3
)


include_directories(
  include
  ${Eigen3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
  
add_library(${PROJECT_NAME}
  src/vehicle_param.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)