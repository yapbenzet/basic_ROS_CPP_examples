#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_left_wheel;
ros::Publisher pub_right_wheel;

// Message receive callback
void recvTwist(const geometry_msgs::TwistConstPtr& msg)
{
  // Declare two message structures to be published
  std_msgs::Float64 left_wheel_speed_msg;
  std_msgs::Float64 right_wheel_speed_msg;

  // Apply differential drive kinematics equations and store the
  // outputs in the message structures
  double v = msg->linear.x;
  double psi_dot = msg->angular.z;

  double W = 1.0;
  double rw = 0.2;

  left_wheel_speed_msg.data = (v - 0.5 * W * psi_dot) / rw;
  right_wheel_speed_msg.data = (v + 0.5 * W * psi_dot) / rw;

  // Publish wheel speed commands
  pub_left_wheel.publish(left_wheel_speed_msg);
  pub_right_wheel.publish(right_wheel_speed_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_drive_kinematics");
  ros::NodeHandle node;

  // Initialize subscriber
  ros::Subscriber sub_twist = node.subscribe("cmd_vel", 1, recvTwist);

  // Initialize command publishers
  pub_left_wheel = node.advertise<std_msgs::Float64>("left_wheel_speed", 1);
  pub_right_wheel = node.advertise<std_msgs::Float64>("right_wheel_speed", 1);

  ros::spin();
  return 0;
}
