cmake_minimum_required(VERSION 2.8.3)
project(tension_plugin)

find_package(catkin REQUIRED COMPONENTS gazebo_dev roscpp message_generation xacro)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime gazebo_dev roscpp
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Declare a C++ library
add_library(tension_plugin
  src/tension_plugin.cc
)
target_link_libraries(tension_plugin
  ${catkin_LIBRARIES}
)
install(TARGETS tension_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
