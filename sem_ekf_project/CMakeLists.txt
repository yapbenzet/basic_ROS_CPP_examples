cmake_minimum_required(VERSION 2.8.3)
project(sem_ekf_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  dynamic_reconfigure
  ugv_course_libs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sem_ekf_project 
			 src/sem_ekf_project.cpp
			 src/SemEkfProject.cpp)
			 
target_link_libraries(sem_ekf_project
  ${catkin_LIBRARIES}
)



add_executable(audibot_joint_pub 
			 src/audibot_joint_pub.cpp
			 src/AudibotJointPub.cpp)

target_link_libraries(audibot_joint_pub
  ${catkin_LIBRARIES}
)


#################################







