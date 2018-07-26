#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "namespace_example");

  // Declare three ROS node handles with different namespace scope
  ros::NodeHandle global_handle;
  ros::NodeHandle private_handle("~");
  ros::NodeHandle arbitrary_handle("some_string");

  // Advertise topics with each one to see the implications of node handle scope
  ros::Publisher pub_global = global_handle.advertise<std_msgs::Float64>("global_topic", 1);
  ros::Publisher pub_private = private_handle.advertise<std_msgs::Float64>("private_topic", 1);
  ros::Publisher pub_arbitrary = arbitrary_handle.advertise<std_msgs::Float64>("arbitrary_topic", 1);

  ros::spin();
}
