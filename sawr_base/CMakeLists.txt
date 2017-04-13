cmake_minimum_required(VERSION 2.8.3)
project(sawr_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  nav_msgs
  dynamixel_sdk
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #/usr/local/include
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(sawr_base src/sawr_base.cpp)
target_link_libraries(sawr_base dxl_x64_cpp ${catkin_LIBRARIES})
