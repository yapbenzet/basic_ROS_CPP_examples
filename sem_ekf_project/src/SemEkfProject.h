// Include guard to prevent multiple declarations
#ifndef SEMEKFPROJECT_H
#define SEMEKFPROJECT_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>


//#include <ublox_msgs/HnrPVT.h>

// Namespace matches ROS package name
namespace sem_ekf_project{

class SemEkfProject
{
public:
  SemEkfProject(ros::NodeHandle n, ros::NodeHandle pn);

private:
  //void recvHeading(const ublox_msgs::HnrPVTConstPtr msg);
  void recvFix(const sensor_msgs::NavSatFixConstPtr& msg);
  void recvCanSpeedMessage(const std_msgs::Float64ConstPtr& msg);
  void republishgps(const sensor_msgs::NavSatFixConstPtr& msg);
  void recvYawRate(const std_msgs::Float64ConstPtr& msg);
  
  
  ros::Subscriber sub_Float64; 
  ros::Subscriber sub_ublox_gps;
  ros::Subscriber sub_ublox_gps2;
  ros::Subscriber sub_ublox_heading;
  ros::Subscriber sub_can_speed;
  ros::Subscriber sub_yaw_rate;
  
  //ros::Publisher pub_speed_converted; 
  ros::Publisher pub_gps_fix;
  ros::Publisher pub_twist;
  

  geometry_msgs::TwistStamped twist_data;

};

}

#endif // SEMEKFPROJECT_H
