 cmake_minimum_required(VERSION 2.8.3)
project(graphslam)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    roslaunch
    cv_bridge
    pcl_conversions
    pcl_ros
    laser_geometry
    sensor_msgs
    unavlib
    message_generation
)

add_message_files(
   FILES
)

add_service_files(
  FILES
  manual_control.srv
  test_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


#PointCloud Registrator

set(pc_registrator_source
    src/pc_registrator/src/PointCloudRegistrator.cpp
    src/pc_registrator/src/PointCloudRegistrator.h
)

set(ndt_source
    src/pc_registrator/src/ndt.cpp
    src/pc_registrator/src/ndt.h
)

add_executable(pc_registrator src/pc_registrator/pc_registrator.cpp ${pc_registrator_source} ${ndt_source})
target_link_libraries(pc_registrator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pc_registrator graphslam_generate_messages_cpp)

