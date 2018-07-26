#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <ugv_course_libs/gps_conv.h>
#include <tf/transform_listener.h>
#include "term_color.h"

UTMCoords ref_point;

std::vector<double> waypoint_lat;
std::vector<double> waypoint_lon;

std::vector<tf::Vector3> local_waypoints;
tf::Vector3 current_rel_position;
tf::TransformListener *listener_;
bool received_twist;

TEST(GpsSimTest, test)
{
  bool passed = false;
  bool rollover = false;

  std::vector<bool> hit_waypoints;
  hit_waypoints.resize(8, false);
  int current_waypoint = 0;

  while (!ros::isShuttingDown() && !received_twist) {
    ros::Duration(0.02).sleep();
  }
  test_info("Received first feedback from simulation, time starts now");

  ros::Time start_time = ros::Time::now();

  while (!ros::isShuttingDown() && !passed && !rollover) {

    ros::Duration total_time = ros::Time::now() - start_time;

    // Monitor for rollover crash
    try {
      tf::StampedTransform transform;
      listener_->lookupTransform("world", "audibot/base_footprint", ros::Time(0), transform);
      current_rel_position.setX(transform.getOrigin().x());
      current_rel_position.setY(transform.getOrigin().y());
      double x = transform.getRotation().getX();
      double y = transform.getRotation().getY();
      if (fabs(x) > 0.2 || fabs(y) > 0.2) {
        test_info("Rollover crash!");
        rollover = true;
      }
    } catch (tf::TransformException &ex) {
      ros::Duration(0.02).sleep();
      continue;
    }

    // Monitor distance from current waypoint
    double dist = (current_rel_position - local_waypoints[current_waypoint]).length();
    if (dist <= 1.0) {
      hit_waypoints[current_waypoint++] = true;
      std::stringstream ss;
      ss.precision(4);
      ss << "Hit waypoint " << current_waypoint << " at t = " << total_time.toSec();
      test_info(ss.str().c_str());
    }

    bool hit_all_waypoints = true;
    for (int i = 0; i < 8; i++) {
      hit_all_waypoints &= hit_waypoints[i];
    }

    if (hit_all_waypoints) {
      double final_time = total_time.toSec();
      std::stringstream ss;
      ss.precision(4);
      ss << "Completed all waypoints in " << (int)floor(final_time / 60.0) << "m" << fmod(final_time, 60.0) << "s";
      test_info(ss.str().c_str());
      passed = true;
    }

    ros::Duration(0.02).sleep();
  }

  ASSERT_TRUE(passed);
}

void initializeWaypoints()
{

  waypoint_lat.resize(8);
  waypoint_lat[0] = 42.851358;
  waypoint_lat[1] = 42.851383;
  waypoint_lat[2] = 42.852443;
  waypoint_lat[3] = 42.852021;
  waypoint_lat[4] = 42.851525;
  waypoint_lat[5] = 42.851344;
  waypoint_lat[6] = 42.850836;
  waypoint_lat[7] = 42.849644;
  waypoint_lon.resize(8);
  waypoint_lon[0] = -83.069485;
  waypoint_lon[1] = -83.069007;
  waypoint_lon[2] = -83.068013;
  waypoint_lon[3] = -83.066888;
  waypoint_lon[4] = -83.067044;
  waypoint_lon[5] = -83.066344;
  waypoint_lon[6] = -83.066440;
  waypoint_lon[7] = -83.066060;

  local_waypoints.resize(8);
  for (int i = 0; i < 8; i++) {
    UTMCoords waypoint(LatLon(waypoint_lat[i], waypoint_lon[i], 0.0));
    local_waypoints[i] = waypoint - ref_point;
  }
}

void recvTwist(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  received_twist = true;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_gps_sim_project");
  ros::NodeHandle node;

  listener_ = new tf::TransformListener;

  double ref_lat, ref_lon;
  node.param("/audibot/gps/ref_lat", ref_lat, 0.0);
  node.param("/audibot/gps/ref_lon", ref_lon, 0.0);
  ref_point = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));
  initializeWaypoints();

  received_twist = false;
  ros::Subscriber sub_twist = node.subscribe("/audibot/twist", 1, recvTwist);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  int result = RUN_ALL_TESTS();

  return 0;
}
