cmake_minimum_required(VERSION 3.0.2)
project(robot4ws_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_loader
  grid_map_msgs
  grid_map_ros
  grid_map_rviz_plugin
  rosbag
  std_msgs
  tf2_ros
  sensor_msgs
  costmap_2d
  tf
  cv_bridge
  robot4ws_perception
  tf2_geometry_msgs
  image_geometry  # Added for color grid mapper
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES gridmap_node_lib
  CATKIN_DEPENDS 
    roscpp 
    grid_map_msgs 
    grid_map_ros 
    sensor_msgs 
    costmap_2d 
    tf2_ros 
    tf2_geometry_msgs
    cv_bridge
    image_geometry
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Base library for gridmap node
add_library(gridmap_node_lib
  src/gridmap_node.cpp
  src/gridmap_node.h
)

target_link_libraries(gridmap_node_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(gridmap_node_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# Main gridmap node executable
add_executable(gridmap_node 
  src/gridmap_node.cpp
)

target_link_libraries(gridmap_node
  gridmap_node_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(gridmap_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# Link states to odom node
add_executable(link_states_to_odom_node 
  src/link_states_to_odom_node.cpp
)

target_link_libraries(link_states_to_odom_node
  ${catkin_LIBRARIES}
)

add_dependencies(link_states_to_odom_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# Color grid mapper executable
add_executable(color_grid_mapper 
  src/perception_layer/color_grid_mapper.cpp
  src/perception_layer/color_grid_mapper.h
)

target_link_libraries(color_grid_mapper
  gridmap_node_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(color_grid_mapper 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# Installation rules
install(
  DIRECTORY config data doc launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS 
  gridmap_node_lib 
  gridmap_node 
  link_states_to_odom_node 
  color_grid_mapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  src/gridmap_node.h
  src/perception_layer/color_grid_mapper.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)