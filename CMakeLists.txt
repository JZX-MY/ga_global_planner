cmake_minimum_required(VERSION 3.0.2)
project(ga_global_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  angles
  COMPONENTS
  costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  nav_core
  roscpp
  std_msgs
  visualization_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  nav_core
  roscpp
  std_msgs
  visualization_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/ga_plan_ros.cpp
   src/ga_trajectory.cpp
   src/ga.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ga_planner src/ga_plan_node.cpp)
add_dependencies(ga_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ga_planner ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(FILES ga_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


