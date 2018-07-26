

#include <ros/ros.h>
#include "SemEkfProject.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "my_ekf_project");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Instantiate node class
  sem_ekf_project::SemEkfProject node(n, pn);

  // Spin and process callbacks
  ros::spin();
}



