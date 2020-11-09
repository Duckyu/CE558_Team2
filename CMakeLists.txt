 cmake_minimum_required(VERSION 2.8.3)
project(CE558_Team2)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    mavros_msgs
    roslaunch
    cv_bridge
    pcl_conversions
    pcl_ros
    laser_geometry
    sensor_msgs
    unavlib
    message_generation
    unavlib
)

add_message_files(
   FILES
   log_data.msg
   points_node.msg
   points_nodes.msg
)

add_service_files(
  FILES
  end.srv
  global_path.srv
  local_path.srv
  init_step.srv
  mission_start.srv
  path_terminate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
#  CE558_Team2
#  CE558_Team2_generate_messages_cpp
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


set(control_source
    src/control/src/control.hpp
    src/control/src/control.cpp
    src/control/src/extras.cpp
    src/control/src/sub_srv_fncs.cpp
)

set(mapper_source
    src/mapper/octomap/octoMapgen.cpp
    src/mapper/octomap/octoMapgen.h
)

set(pathgen_source
    src/pathgen/local_pc_registration/local_pc_registration.cpp
    src/pathgen/local_pc_registration/local_pc_registration.hpp
#    src/pathgen/normal_path_sampler/normal_path_sampler.hpp
#    src/pathgen/normal_path_sampler/normal_path_sampler.hpp
)

# set(octomap_source
#    src/pathgen/octomap/octoMapgen.cpp
#    src/pathgen/octomap/octoMapgen.hpp
# )

add_executable(control src/control/UAV_control.cpp ${control_source})
add_dependencies(control CE558_Team2_generate_messages_cpp)
target_link_libraries(control
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

#add_executable(CE558_mapper src/mapper/mapper.cpp ${mapper_source})
#add_dependencies(CE558_mapper CE558_Team2_generate_messages_cpp)
#target_link_libraries(CE558_mapper
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#)

add_executable(CE558_pathgen src/pathgen/pathgen.cpp ${pathgen_source})
add_dependencies(CE558_pathgen CE558_Team2_generate_messages_cpp)
target_link_libraries(CE558_pathgen
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
