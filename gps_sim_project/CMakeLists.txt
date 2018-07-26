cmake_minimum_required(VERSION 2.8.3)
project(gps_sim_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
  std_msgs
  ugv_course_libs
)

catkin_package()

add_executable(gps_sim_project src/gps_sim_project.cpp)
target_link_libraries(gps_sim_project ${catkin_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#################
## Test Script ##
#################
add_executable(test_gps_sim_project test/test_gps_project.cpp)
target_link_libraries(test_gps_sim_project ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
