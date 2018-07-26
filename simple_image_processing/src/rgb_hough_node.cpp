#include <ros/ros.h>
#include "RgbHough.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgb_hough_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  simple_image_processing::RgbHough node(n, pn);
  
  ros::Rate loop_rate(30);
	 while (n.ok()) 
	 {
       ros::spinOnce();
       loop_rate.sleep();
     }

  //ros::spin();
}
