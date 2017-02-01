cmake_minimum_required(VERSION 2.8.3)
project(mesh_navigation)

set(PACKAGE_DEPENDENCIES 
  roscpp
  robot_navigation
  std_msgs
  pluginlib
  dynamic_reconfigure
  mesh_msgs
  mesh_msgs_transform
  tf
  lvr_ros
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL REQUIRED)
find_package(lvr REQUIRED)

add_definitions(${PCL_DEFINITIONS})
add_definitions(${lvr_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${lvr_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${lvr_LIBRARY_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/MeshLocalPlanner.cfg
  cfg/MeshGlobalPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS PCL lvr
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${lvr_LIBRARIES}
)

add_library(mesh_global_planner
  src/mesh_global_planner.cpp
  src/mesh_planner_plugins.cpp)
 
add_library(mesh_local_planner
  src/mesh_local_planner.cpp
  src/mesh_planner_plugins.cpp)

add_dependencies(mesh_global_planner ${PROJECT_NAME}_gencfg)
add_dependencies(mesh_local_planner ${PROJECT_NAME}_gencfg)

target_link_libraries(mesh_global_planner ${LINK_LIBS})
target_link_libraries(mesh_local_planner ${LINK_LIBS})

install(TARGETS mesh_global_planner mesh_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES mesh_planner_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
