cmake_minimum_required(VERSION 2.8.3)
project(ekf_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ugv_course_libs
  tf
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/EkfExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf_example
  src/ekf_example.cpp
  src/EkfExample.cpp  
)
add_dependencies(ekf_example
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(ekf_example
  ${catkin_LIBRARIES}
)
