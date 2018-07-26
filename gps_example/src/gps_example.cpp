#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>

// Store the reference coordinates in UTM
UTMCoords ref_coords;

// Position of the robot relative to the reference coordinates
tf::Vector3 relative_position;

// Path message for publishing to Rviz
nav_msgs::Path gps_path;

// ROS publisher
ros::Publisher pub_path;

// Receive callback for raw GPS messages in geodetic coordinates
void recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
  // Convert current geodetic coordinates into UTM
  UTMCoords current_utm(*msg);
  
  // Use overloaded - operator to subtract two UTM objects to 
  // get the relative position as a tf::Vector3
  relative_position = current_utm - ref_coords;
  
  ROS_INFO("Current UTM: (%f, %f)", current_utm.getX(), current_utm.getY());
  ROS_INFO("Position relative to ref: (%f, %f)", relative_position.x(), relative_position.y());
}

// Timer callback to add latest position to the end of the path,
// update time stamps, and publish the path message
void timerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::PoseStamped new_path_point;
  new_path_point.pose.position.x = relative_position.x();
  new_path_point.pose.position.y = relative_position.y();
  
  gps_path.poses.push_back(new_path_point);
  
  gps_path.header.stamp = event.current_real;
  pub_path.publish(gps_path);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_example");
  ros::NodeHandle node;

  // Set up topics and timer
  ros::Subscriber sub_fix = node.subscribe("/audibot/gps/fix", 1, recvFix);
  ros::Timer path_timer = node.createTimer(ros::Duration(0.5), timerCallback);
  pub_path = node.advertise<nav_msgs::Path>("gps_path", 1);
  
  // Read reference coordinates from the parameter server as lat/lon
  double ref_lat;
  double ref_lon;
  node.param("/audibot/gps/ref_lat", ref_lat, 0.0);
  node.param("/audibot/gps/ref_lon", ref_lon, 0.0);
  
  // Instantiate a LatLon instance with the reference geodetic coordinates
  LatLon ref_lat_lon(ref_lat, ref_lon, 0.0);

  // Pass the LatLon object instance to the UTMCoords constructor to convert
  // to UTM and store the resulting object in the global instance.
  ref_coords = UTMCoords(ref_lat_lon);
  
  // Set frame ID of the path for Rviz to render it properly
  gps_path.header.frame_id = "world";
  
  ros::spin();
}
