#include <ros/ros.h>

// Callback function that runs every time the timer is triggered
void timerCallback(const ros::TimerEvent& event)
{
  // Output the actual time when the timer trigger happened, and the time it was scheduled to happen
  ROS_INFO("current time: %f", event.current_real.toSec());
  ROS_INFO("expected time: %f", event.current_expected.toSec());

  ros::Duration time_since_last_real = event.current_real - event.last_real;
  ros::Duration time_since_last_expected = event.current_expected - event.last_expected;

  ROS_INFO("Actual delta t: %f, Expected delta t: %f", time_since_last_real.toSec(), time_since_last_expected.toSec());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "timer_example");
  ros::NodeHandle node;

  // Initialize the timer to trigger every 0.5 seconds
  ros::Timer timer = node.createTimer(ros::Duration(0.5), timerCallback);

  ros::spin();
}
