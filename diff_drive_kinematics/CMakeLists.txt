cmake_minimum_required(VERSION 2.8.3)
project(diff_drive_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(diff_drive_kinematics src/diff_drive_kinematics.cpp)
target_link_libraries(diff_drive_kinematics ${catkin_LIBRARIES})
