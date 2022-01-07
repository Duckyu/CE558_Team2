cmake_minimum_required(VERSION 2.8.3)
project(n_cpp)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
add_compile_options(-std=c++17)

find_package(PythonLibs 2.7 REQUIRED)

# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_utils REQUIRED)
find_package(mpl_external_planner REQUIRED)
find_package(planning_ros_utils REQUIRED)
include_directories(${planning_ros_utils_INCLUDE_DIRS} ${decomp_ros_utils_INCLUDE_DIRS} ${mpl_external_planner_INCLUDE_DIRS} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS}) #${EIGEN3_INCLUDE_DIR} 


# catkin_simple()#DUCK

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
    message_generation
    message_filters
    
)

add_message_files(
   FILES
   log_data.msg
   point_node.msg
  #  point_nodes.msg
)

add_service_files(DIRECTORY srv
  FILES
  # add_node.srv
  # end.srv
  # global_path.srv
  # local_path.srv
  # init_step.srv
  # mission_start.srv
  go_get_it.srv
  path_require.srv
  NextStep.srv
  RayCast.srv
  EliminateTriple.srv
  ThreeDAstar.srv
  FindCollisionAvoidPath.srv
  get_octomap.srv
  get_triple_points.srv
  collision_check.srv
  mission_terminate.srv
)

#catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  planning_ros_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/ncpp_dualshock.py scripts/UAV2map_pub_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(tester src/tester.cpp )
add_dependencies(tester n_cpp_generate_messages_cpp)
target_link_libraries(tester
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)


include(${VTK_USE_FILE})
add_executable(n_ref_mesh src/mesh2pc/mesh_sampling.cpp)
add_dependencies(n_ref_mesh n_cpp_generate_messages_cpp)
target_link_libraries(n_ref_mesh ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${VTK_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES})

set(n_octomap_source
    src/mapper/src/octomap.hpp 
    src/mapper/src/octo_general.cpp 
    src/mapper/src/octo_input.cpp 
    src/mapper/src/octo_subs.cpp 
    src/mapper/src/octo_srvs.cpp 
    src/mapper/src/new_octo_srvs.cpp
)

add_executable(n_octomap src/mapper/octomap.cpp ${n_octomap_source})
add_dependencies(n_octomap n_cpp_generate_messages_cpp)
target_link_libraries(n_octomap ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES})

set(n_pathcand_source
    src/path/src/normal_extraction.cpp
    src/path/src/new_normal_extraction.cpp
    src/path/src/path_subs.cpp
    src/path/src/path_srvs.cpp
    src/path/src/pc_general.cpp
)

set(n_pathgen_source
    src/path/src/path_gen.hpp
    # src/path/src/path_align.cpp
#     src/path/src/cand_elimination.cpp
)

add_executable(n_path src/path/path_gen.cpp ${n_pathcand_source} ${n_pathgen_source})
add_dependencies(n_path n_cpp_generate_messages_cpp)
target_link_libraries(n_path ${DECOMP_UTIL_INCLUDE_DIRS} ${MOTION_PRIMITIVE_LIBRARY_INCLUDE_DIRS} ${EIGEN3_LIBS} ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES})
