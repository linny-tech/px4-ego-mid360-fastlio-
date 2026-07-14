cmake_minimum_required(VERSION 3.0.2)
project(offboard_hover)

add_compile_options(-std=c++14 -Wall -Wextra)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  tf2
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(offboard_hover_node src/offboard_hover_node.cpp)
add_dependencies(offboard_hover_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(offboard_hover_node
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
