cmake_minimum_required(VERSION 3.1.3)
project(visualization_tools)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  roscpp
  goals_sequence_path_planner
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  goals_sequence_path_planner
  std_msgs
  visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(topics_advertiser src/advertise_topics_node.cpp)
add_executable(visualization src/visual_rviz_node.cpp
                             src/visual_rviz.cpp
)

target_link_libraries(topics_advertiser  ${catkin_LIBRARIES})
target_link_libraries(visualization  ${catkin_LIBRARIES})
