cmake_minimum_required(VERSION 3.0.2)
project(fastlio_px4_bridge)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(fastlio_px4_bridge_node src/fastlio_px4_bridge_node.cpp)
add_dependencies(fastlio_px4_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastlio_px4_bridge_node ${catkin_LIBRARIES})
