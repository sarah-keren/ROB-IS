 
cmake_minimum_required(VERSION 2.8.3)
project(rosplan_waypoint_sampling)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosplan_knowledge_msgs
  nav_msgs
  std_srvs
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
  occupancy_grid_utils
  message_generation
  genmsg
  rosplan_interface_mapping
  grid_map_msgs
  grid_map_ros
  eigen_conversions
  eigen_stl_containers
  rviz_visual_tools
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# waypoint filter
add_executable(rp_roadmap_filter src/RPRoadmapFilter.cpp)
add_executable(rp_fixed_wp_gen src/RPFixedWaypointGenerator.cpp include/robis_interface_mapping/RPFixedWaypointGenerator.h)
target_link_libraries(rp_roadmap_filter ${catkin_LIBRARIES})
target_link_libraries(rp_fixed_wp_gen ${catkin_LIBRARIES})

#rp_roadmap_server_pref
add_executable(rp_roadmap_server_pref src/RPRoadmapServerPref.cpp include/robis_interface_mapping/RPRoadmapServerPref.h)
target_link_libraries(rp_roadmap_server_pref ${catkin_LIBRARIES})



