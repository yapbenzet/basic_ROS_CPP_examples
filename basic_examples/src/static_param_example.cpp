#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_param_example");
  ros::NodeHandle global_nh;
  ros::NodeHandle private_nh("~");
  
  double p1;
  global_nh.param("p1", p1, 1.0);
  ROS_INFO("P1 value: %f", p1);
  
  double p2;
  bool found_p2 = global_nh.getParam("p2", p2);
  if (!found_p2) {
    ROS_WARN("P2 not found!");
  }
  ROS_INFO("P2 value: %f", p2);
  
  double p3;
  private_nh.param("p3", p3, 1.0);
  ROS_INFO("P3 value: %f", p3);
    
  ros::spin();
}
