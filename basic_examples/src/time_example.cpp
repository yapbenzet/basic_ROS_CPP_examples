#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_example");
  ros::NodeHandle node;

  // Take a snapshot of the current system time
  ros::Time current_time = ros::Time::now();

  // Define a 60 second period of time in a ros::Duration object
  ros::Duration one_minute(60.0);

  // Compute the absolute UNIX time for one minute into the future
  ros::Time one_minute_later = current_time + one_minute;

  // Compute the time difference between two absolute time stamps
  ros::Duration diff_time = one_minute_later - current_time;

  // Express the current time's value as a floating point number
  double current_time_value = current_time.toSec();

  ROS_INFO("now: %f, later: %f", current_time_value, one_minute_later.toSec());
  ROS_INFO("diff: %f", diff_time.toSec());
  
  return 0;
}
