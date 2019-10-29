cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(kinect_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(INCLUDE_DIRS include)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(kinect_teleop src/kinect_teleop.cpp)
target_link_libraries(kinect_teleop ${catkin_LIBRARIES})
add_dependencies(kinect_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

