cmake_minimum_required(VERSION 2.8.3)
project(odom)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odom
#  CATKIN_DEPENDS nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  src/stream/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(stream
  src/stream/serial.cc
  src/stream/socket.cc
)

add_executable(node src/odom_node.cpp)

target_link_libraries(node
  stream
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
