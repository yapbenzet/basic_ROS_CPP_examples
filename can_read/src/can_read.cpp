

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include<tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <src/PCANBasic.h>
#include </usr/include/x86_64-linux-gnu/asm/types.h>
#include <can_read/custom_can_message.h>
#include <sstream>

ros::Publisher pub_can_messages;
ros::Publisher pub_speed_message;
ros::Publisher pub_speed_can_message;

TPCANMsg Message;
TPCANStatus Status;

float scale_factor = 0.01;
float offset = 0.0;



int main(int argc, char** argv)
{
	ros::init(argc, argv, "can_read");
	ros::NodeHandle nh;
	
	unsigned long ulIndex = 0;

	Status = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0);
	ROS_INFO("Initialize CAN: %i\n",(int)Status);
	
	ros::Duration sleep_time = ros::Duration(0.001);
	
	pub_can_messages = nh.advertise<can_read::custom_can_message>("can_messsages",1);
	
	pub_speed_message = nh.advertise<std_msgs::Float64>("speed_messsage",1);
	
	pub_speed_can_message = nh.advertise<can_read::custom_can_message>("speed_can_messsage",1);
	
	while( ros::ok())
    {
	 
	 while((Status=CAN_Read(PCAN_USBBUS1,&Message,NULL)) == PCAN_ERROR_QRCVEMPTY)
	 {
	      sleep_time.sleep();
	 }

	 //ROS_INFO("CAN_Read Error status is %x",(int)Status);
	 

	  if (Status != PCAN_ERROR_OK) 
	  {
	       ROS_INFO("Error 0x%x",(int)Status);
	  }
	  
	/*** PLEASE DONT REMOVE FOLLOWING PORTION	***/		 
	/*** PLEASE DONT REMOVE FOLLOWING PORTION	***/
	/*** PLEASE DONT REMOVE FOLLOWING PORTION	***/
	/*  
	   * ROS_INFO("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
			 (int)Message.ID, (int)Message.LEN,
			 (int)Message.DATA[0], (int)Message.DATA[1],
			 (int)Message.DATA[2], (int)Message.DATA[3],
			 (int)Message.DATA[4], (int)Message.DATA[5],
			 (int)Message.DATA[6], (int)Message.DATA[7]);
	  
	 */
	  
	  can_read::custom_can_message new_can_message;
	  new_can_message.message_id = (int)Message.ID;
	  new_can_message.message_length = (int)Message.LEN;
	  new_can_message.byte0 = (int)Message.DATA[0];
	  new_can_message.byte1 = (int)Message.DATA[1];
	  new_can_message.byte2 = (int)Message.DATA[2];
	  new_can_message.byte3 = (int)Message.DATA[3];
	  new_can_message.byte4 = (int)Message.DATA[4];
	  new_can_message.byte5 = (int)Message.DATA[5];
	  new_can_message.byte6 = (int)Message.DATA[6];
	  new_can_message.byte7 = (int)Message.DATA[7]; 
	  
	  
	  
	  // Extract speed from 0x202
	  if((int)Message.ID==0x202)		// I know this is the message that has speed, I reverse engineered it..!
	  {
	       
	       // Speed is in kph and varies from 0x0000 to 0xFFFF -> 0 to 655.35 kph
	       // combine 2 bytes here, use scaling factor and offset for conversion
	       int speed_2_bytes_cobined = 256*new_can_message.byte6 + new_can_message.byte7;
	       
	       // Populate and publish standard float message 
	       std_msgs::Float64 vehicle_speed;
	       vehicle_speed.data = (speed_2_bytes_cobined*scale_factor + offset)*5/18;
	       pub_speed_message.publish(vehicle_speed);
	       
	       // for debug
	       pub_speed_can_message.publish(new_can_message);
	       
	  }
	  
	  // Publish all CAN messages read on bus
	  pub_can_messages.publish(new_can_message);
	  
	  ros::spinOnce();
    }
}