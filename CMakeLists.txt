cmake_minimum_required(VERSION 2.8.3)
project(mesh_navigation)

set(PACKAGE_DEPENDENCIES 
  roscpp
  nav_msgs
  nav_core
  std_msgs
  pluginlib
  dynamic_reconfigure
  mesh_msgs
  lvr_ros
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)


add_definitions("-frounding-math")
add_definitions("-std=c++0x")
add_definitions(${PCL_DEFINITIONS})
add_definitions(-D_USE_PCL_)


FIND_PATH(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

find_package(PCL 1.0 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

### LVR
set(LVR_DIR "/home/robot/ros/muffin-dry/mapping/lvr")
set(LVR_INCLUDE_DIRS
  ${LVR_DIR}/ext/psimpl
  ${LVR_DIR}/ext/rply
  ${LVR_DIR}/ext/stann
  ${LVR_DIR}/ext
  ${LVR_DIR}/include/liblvr
)
set(LVR_LIBRARY_DIRS
  ${LVR_DIR}/lib
)
set(_LVR_LIBRARIES
  lvr
)
foreach(lib ${_LVR_LIBRARIES})
  find_library(${lib}_FOUND ${lib} HINTS ${LVR_LIBRARY_DIRS})
  if(${lib}_FOUND)
    list(APPEND LVR_LIBRARIES ${${lib}_FOUND})
  else()
    message( FATAL_ERROR "${lib} not found" )
  endif()
endforeach()

### end lvr

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${lvr_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${LVR_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${CGAL_LIBRARY_DIRS}
  ${OPENGL_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  ${LVR_LIBRARY_DIRS}
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${LVR_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/MeshLocalPlanner.cfg
  cfg/MeshGlobalPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS LVR OPENGL GLUT CGAL LVR
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  ${LVR_LIBRARIES}
  ${OPENGL_glu_LIBRARY}
)

add_library(mesh_global_planner src/mesh_global_planner.cpp)
add_library(mesh_local_planner src/mesh_local_planner.cpp)

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

install(FILES mesh_path_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
