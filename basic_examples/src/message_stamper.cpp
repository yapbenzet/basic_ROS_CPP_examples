#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

ros::Publisher pub_twist_stamped;

// Message receive callback function
void recvTwist(const geometry_msgs::TwistConstPtr& msg)
{
  // Declare message structure that will be published
  geometry_msgs::TwistStamped twist_stamped_msg;

  // Populate header with current time and blank frame ID
  twist_stamped_msg.header.stamp = ros::Time::now();
  twist_stamped_msg.header.frame_id = "";

  // Populate twist data payload with the incoming data
  twist_stamped_msg.twist = *msg;

  // Publish
  pub_twist_stamped.publish(twist_stamped_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_stamper");
  ros::NodeHandle node;

  // Initialize subscriber
  ros::Subscriber sub_twist = node.subscribe("twist", 1, recvTwist);

  // Initialize publisher
  pub_twist_stamped = node.advertise<geometry_msgs::TwistStamped>("twist_stamped", 1);

  ros::spin();
}
