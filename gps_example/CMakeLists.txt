cmake_minimum_required(VERSION 2.8.3)
project(gps_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  dynamic_reconfigure
  ugv_course_libs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(gps_example src/gps_example.cpp)
target_link_libraries(gps_example ${catkin_LIBRARIES})

