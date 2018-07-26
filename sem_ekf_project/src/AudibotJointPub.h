// Include guard to prevent multiple declarations
#ifndef AUDIBOTJOINTPUB_H
#define AUDIBOTJOINTPUB_H

// ROS header
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

// Namespace matches ROS package name
namespace audibot{

class AudibotJointPub
{
public:
  AudibotJointPub(ros::NodeHandle n, ros::NodeHandle pn);
  
private:
  void timerCallback(const ros::TimerEvent& event);
  
  ros::Publisher pub_joint_state;
  ros::Timer timer;
  
  double joint_angle;
  
  sensor_msgs::JointState joint_state_msg;
  
};

}

#endif // AUDIBOTJOINTPUB_H

