

#include "SemEkfProject.h"



namespace sem_ekf_project {

SemEkfProject::SemEkfProject(ros::NodeHandle n, ros::NodeHandle pn)
{
  
  sub_Float64 = n.subscribe("/speed_messsage", 1, &SemEkfProject::recvCanSpeedMessage, this);
  
  sub_ublox_gps = n.subscribe("/n_nmea/fix",1, &SemEkfProject::recvFix, this);
  
  sub_yaw_rate = n.subscribe("/yaw_rate", 1, &SemEkfProject::recvYawRate, this);
  
  //sub_ublox_heading = n.subscribe("/n_nmea/hnrpvt",1, &SemEkfProject::recvHeading, this);
  
  //pub_speed_converted = n.advertise<std_msgs::Float64>("/speed_messsage_converted",1);
  
  pub_twist = n.advertise<geometry_msgs::TwistStamped>("/twist", 1);
  
  pub_gps_fix = n.advertise<sensor_msgs::NavSatFix>("/gps_fix", 1);
  
  
  
}

/*
void SemEkfProject::recvHeading(const ublox_msgs::HnrPVTConstPtr msg)
{
 double psi_raw  = msg->headMot;
 
 twist_data.angular.z = psi_raw*0.00001;

}
*/

void SemEkfProject::recvYawRate(const std_msgs::Float64ConstPtr& msg)
{
     twist_data.twist.angular.z = msg->data;
     
}

void SemEkfProject::recvCanSpeedMessage(const std_msgs::Float64ConstPtr& msg)
{
	std_msgs::Float64 new_Float64;
     new_Float64.data = msg->data;
   
     
     
     //Ask Prof
     twist_data.header.frame_id = "base_footprint";		//base footprint of robot	
     twist_data.header.stamp = ros::Time::now();
     
	twist_data.twist.linear.x = msg->data;
	pub_twist.publish(twist_data);
}



void SemEkfProject::recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
  sensor_msgs::NavSatFix new_msg;
  new_msg = *msg;
  
  // Update header of gps data just to ensure frame name and time are updated
  new_msg.header.frame_id = "ublox_gps";
  new_msg.header.stamp = ros::Time::now();
  
  // Publish modified NavSatFix message that is to be used by Kalman filter and other nodes
  pub_gps_fix.publish(new_msg);

}



}
