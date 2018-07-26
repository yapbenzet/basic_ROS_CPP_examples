cmake_minimum_required(VERSION 2.8.3)
project(can_read)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
  std_msgs
  ugv_course_libs
  message_generation
)

add_message_files(FILES
                  custom_can_message.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)




add_executable(can_read src/can_read.cpp)
target_link_libraries(can_read ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libpcanbasic.so)

include_directories(can_read ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/PCANBasic.h)

