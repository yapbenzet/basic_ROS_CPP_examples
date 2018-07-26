#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <basic_examples/ReconfigExampleConfig.h>

// Global variable to access the config data from elsewhere in the program
basic_examples::ReconfigExampleConfig cfg;

// Dynamic reconfigure server callback function runs once when a GUI change is detected
void reconfig(basic_examples::ReconfigExampleConfig& config, uint32_t level)
{
  ROS_INFO("Current values:");
  if (config.enable) {
    ROS_INFO("Enabled");
  } else {
    ROS_INFO("Disabled");
  }
  
  if (config.x > 50) {
    config.x = 50;
  }
  
  ROS_INFO("X: %f", config.x);
  ROS_INFO("Y: %f", config.y);
  
  ROS_INFO("String: %s", config.str.c_str());
  
  switch (config.list) {
    case 0:
      ROS_INFO("Option 1 selected");
      break;
    case 1:
      ROS_INFO("Option 2 selected");
      break;
    case 2:
      ROS_INFO("Option 3 selected");
      break;      
  }
  
  // Copy config data into global variable
  cfg = config;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_param_example");
  ros::NodeHandle global_nh;
  ros::NodeHandle private_nh("~");
  
  // Declare and instantiate server instance
  dynamic_reconfigure::Server<basic_examples::ReconfigExampleConfig> srv;

  // Bind the callback function, and run it once with the current values in the GUI
  srv.setCallback(boost::bind(reconfig, _1, _2));
  
  ros::spin();
}
