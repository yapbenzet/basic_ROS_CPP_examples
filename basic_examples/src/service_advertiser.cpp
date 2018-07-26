#include <ros/ros.h>
#include <basic_examples/Adder.h>

// Service callback function that implements the desired behavior of the service
bool serviceCallback(basic_examples::Adder::Request& req, basic_examples::Adder::Response& res)
{
  res.result = req.val1 + req.val2;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "service_advertiser");
  ros::NodeHandle node;

  // Initialize service server
  ros::ServiceServer srv = node.advertiseService("/adder_service", serviceCallback);

  ros::spin();
}
