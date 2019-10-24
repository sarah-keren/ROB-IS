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
target_link_libraries(rp_roadmap_filter ${catkin_LIBRARIES})
