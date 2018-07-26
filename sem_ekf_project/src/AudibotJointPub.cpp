// Header file for the class
#include "AudibotJointPub.h"

// Namespace matches ROS package name
namespace audibot {

// Constructor with global and private node handle arguments
AudibotJointPub::AudibotJointPub(ros::NodeHandle n, ros::NodeHandle pn)
{
  
  joint_angle = 0;
  
  pub_joint_state = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  timer = n.createTimer(ros::Duration(0.02), &AudibotJointPub::timerCallback, this);
  
  joint_state_msg.position.resize(4);
  joint_state_msg.name.resize(4);
  joint_state_msg.name[0] = "left_front_wheel_joint";
  joint_state_msg.name[1] = "right_front_wheel_joint";
  joint_state_msg.name[2] = "left_rear_wheel_joint";
  joint_state_msg.name[3] = "right_rear_wheel_joint";
 
}

void AudibotJointPub::timerCallback(const ros::TimerEvent& event)
{
  double constant_speed = 1.0;
  
  joint_state_msg.header.stamp = event.current_real;
  joint_angle += 0.02 * constant_speed;
  joint_state_msg.position[0] = joint_angle;
  joint_state_msg.position[1] = -joint_angle;
  joint_state_msg.position[2] = joint_angle;
  joint_state_msg.position[3] = -joint_angle;
  
  pub_joint_state.publish(joint_state_msg);
}
  
}
