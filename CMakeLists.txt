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
  init_exploration.srv
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


set(control_source
    src/control/src/control.hpp
    src/control/src/control.cpp
    src/control/src/extras.cpp
    src/control/src/sub_srv_fncs.cpp
)

set(mapper_source
    src/mapper/src/global_pc_registration.cpp
    src/mapper/src/global_pc_registration.hpp
)

set(pathgen_source
    src/pathgen/local_pc_registration/local_pc_registration.cpp
    src/pathgen/local_pc_registration/local_pc_registration.hpp
)

# set(octomap_source
#    src/pathgen/octomap/octoMapgen.cpp
#    src/pathgen/octomap/octoMapgen.hpp
# )

add_executable(CE558_control src/control/UAV_control.cpp ${control_source})
add_dependencies(CE558_control CE558_Team2_generate_messages_cpp)
target_link_libraries(CE558_control
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(CE558_mapper src/mapper/mapper.cpp ${mapper_source})
add_dependencies(CE558_mapper CE558_Team2_generate_messages_cpp)
target_link_libraries(CE558_mapper
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(CE558_pathgen src/pathgen/pathgen.cpp ${pathgen_source})
add_dependencies(CE558_pathgen CE558_Team2_generate_messages_cpp)
target_link_libraries(CE558_pathgen
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)