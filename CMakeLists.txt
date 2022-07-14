cmake_minimum_required(VERSION 2.8.3)
project(l_shape_fitting)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  pcl_ros
  roscpp
  tf2
  tf2_geometry_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(l_shape_fitting
  src/main.cpp
  src/shape_estimator.cpp
)

add_dependencies(l_shape_fitting
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(l_shape_fitting
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)