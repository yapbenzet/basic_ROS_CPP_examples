#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub_markers;

visualization_msgs::MarkerArray marker_msg;

void timerCallback(const ros::TimerEvent& event)
{
  if (marker_msg.markers.size() == 0) {
    return;
  }
  
  // Put latest time stamp on each marker before publishing
  marker_msg.markers[0].header.stamp = event.current_real;
  marker_msg.markers[1].header.stamp = event.current_real;
  pub_markers.publish(marker_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_example");
  ros::NodeHandle n;
  
  pub_markers = n.advertise<visualization_msgs::MarkerArray>("markers", 1);
  ros::Timer marker_timer = n.createTimer(ros::Duration(0.05), timerCallback);
  
  // Allocate two marker elements in the marker array message
  marker_msg.markers.resize(2);
  
  // Declare a marker and populate it
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;

  marker.scale.x = 1.0;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  
  marker.pose.position.x = 2.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
 
  // IDs for each marker in an array must be unique 
  marker.id = 0;
  
  marker_msg.markers[0] = marker;
  
  marker.type = visualization_msgs::Marker::CUBE;
  marker.header.frame_id = "frame1";
  marker.color.b = 1.0;
  marker.scale.x = 0.3;
  marker.id = 1;
  
  marker_msg.markers[1] = marker;
  
  ros::spin();
}
