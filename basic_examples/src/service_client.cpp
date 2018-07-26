#include <ros/ros.h>
#include <basic_examples/Adder.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "service_client");
  ros::NodeHandle node;

  // Initialize service client
  ros::ServiceClient srv = node.serviceClient<basic_examples::Adder>("/adder_service");

  // Declare request and response structures to be passed to the service server
  basic_examples::AdderRequest req;
  basic_examples::AdderResponse res;

  // Populate the request with the data the server needs
  req.val1 = 20;
  req.val2 = 40;

  // Wait until the service server is running and ready to process calls
  srv.waitForExistence();

  // Call the service and store the success / failure in a boolean variable
  bool status = srv.call(req, res);

  // Output the result or a warning to the terminal
  if (status) {
    ROS_INFO("%f", res.result);
  } else {
    ROS_WARN("Service call failed");
  }

  return 0;
}
