

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include<tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include<visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

// Number of waypoints
#define n_waypoints 8



/**********	All publisher here	**********/

// Global publisher to publish path points
ros::Publisher pub_path_points;

ros::Publisher pub_waypoints_marker;

ros::Publisher pub_speed_steering;





/**********	Global Params here	**********/

// Current heading angle of audibot in world frame with X-axis
double psi;

// PD-control params for speed and steering control
double Kp_speed = 1.6;
double Kd_speed = 1.0;

double Kp_angle = 2.3;
double Kd_angle = 1.0;



/**********	Global objects of class	**********/

// Store the reference coordinates in UTM
UTMCoords ref_coords;

// Position of the robot relative to the reference coordinates
tf::Vector3 relative_position;

// Array of Position of gps waypoints 
tf::Vector3 relative_position_gps_waypoint[n_waypoints];

// Marker array to show GPS waypoints
visualization_msgs::MarkerArray gps_waypoints_array;

// Path message object for publishing to Rviz
nav_msgs::Path gps_path;

// Marker to set properties of and setup each marker as GPS waypoint
visualization_msgs::Marker marker;

// Transform for audibot translation and rotation
tf::Transform transform;


/**********	Function Declrations	**********/

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg);

void timerCallback(const ros::TimerEvent& event);

void headingCallBackFunc(std_msgs::Float64 current_heading);

void speedSteeringControl();



// Receive callback for raw GPS messages in geodetic coordinates
void recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
	// Convert current geodetic coordinates into UTM
	UTMCoords current_utm(*msg);
	
	// Use overloaded - operator to subtract two UTM objects to 
	// get the relative position as a tf::Vector3
	relative_position = current_utm - ref_coords;
	
	//ROS_INFO("Current UTM: (%f, %f)", current_utm.getX(), current_utm.getY());
	//ROS_INFO("Position relative to ref: (%f, %f)", relative_position.x(), relative_position.y());
}

// Timer callback to add latest position to the end of the path,
// update time stamps, and publish the path message
void timerCallback(const ros::TimerEvent& event)
{

	// Create static broadcaster that creates node handle only once and then used to broadcast transform all the times
	static tf::TransformBroadcaster broadcaster;
	
	if (gps_waypoints_array.markers.size()==0)
	{
	ROS_INFO_STREAM("Markers are yet to be displayed..!");
	return;	// for first few initial calls until when markers dont exist
	}


	// Create and update gloabl reference transform frame
	tf::StampedTransform transform;

	transform.frame_id_="world";
	transform.child_frame_id_="/audibot/gps_waypoints_marker";

	//Set position of transform 
	transform.setOrigin(tf::Vector3(0,0,0));

	//Set orientation of transform
	transform.setRotation(tf::createQuaternionFromRPY(0.0,0.0,0.0));

	//Set stamp by reading current time
	transform.stamp_ = event.current_real;

	// PoseStamped geometry message to update path audibot is currently at
	geometry_msgs::PoseStamped new_path_point;
	new_path_point.pose.position.x = relative_position.x();
	new_path_point.pose.position.y = relative_position.y();
	gps_path.poses.push_back(new_path_point);
	gps_path.header.frame_id = "world";
	gps_path.header.stamp = event.current_real;

	// broadcast trasnform 
	broadcaster.sendTransform(transform);

	// Publish path message
	pub_path_points.publish(gps_path);

	// Publish marker array message
	pub_waypoints_marker.publish(gps_waypoints_array);
	
	// Control speed and steering continuously after specific time interval
	speedSteeringControl();
	
}



void speedSteeringControl()
{
	static bool track_completed = false;
	
	static int next_gps_waypoint = 0;
	
	double audibot_speed;
	double audibot_speed_max = 170.0;
	double audibot_speed_min = 10.50;
	double steering_wheel_angle = 0.0;
	double steering_wheel_angle_max = 10.0;
	double distance = 0.0;
	double target_angle = 0.0;
	
	
	
	// Translation from G to V
	tf::Vector3 T_GV = tf::Vector3(relative_position.x(), relative_position.y(), 0.0);

	// Rotation from G to V
	tf::Quaternion q_GV = tf::createQuaternionFromYaw(psi);

	// Populate tf::Transform object with translation and rotation information
	transform.setOrigin(T_GV);
	transform.setRotation(q_GV);

	// Store data from incoming geometry_msgs::Point into a tf::Vector3
	tf::Vector3 target_global(relative_position_gps_waypoint[next_gps_waypoint].x(), \
								relative_position_gps_waypoint[next_gps_waypoint].y(), 0.0);
	

	// Use tf to apply the inverse transformation
	tf::Vector3 target_vehicle = transform.inverse() * target_global;
	
	distance = sqrt((target_vehicle.x()*target_vehicle.x()) + (target_vehicle.y()*target_vehicle.y()));
	
	
	
	target_angle = atan(target_vehicle.y()/target_vehicle.x());
	
	if((target_vehicle.y()<0.0)&&(target_vehicle.x()<0.0))
	{
		target_angle = atan(target_vehicle.y()/target_vehicle.x()) - M_PI;//M_PI - atan(target_vehicle.y()/target_vehicle.x());
	}
	else if((target_vehicle.y()<0.0)&&(target_vehicle.x()>0.0))
	{
		target_angle = atan(target_vehicle.y()/target_vehicle.x());
	}
	else if((target_vehicle.y()>0.0)&&(target_vehicle.x()<0.0))
	{
		target_angle = atan(target_vehicle.y()/target_vehicle.x()) + M_PI;//-atan(target_vehicle.y()/target_vehicle.x());
	}
	else
	{
		target_angle = atan(target_vehicle.y()/target_vehicle.x());
	}
	
	//ROS_INFO("Target X Distance is: %f, Target Y Distance is: %f, Target angle is: %f",\
			target_vehicle.x(), target_vehicle.y(),(target_angle*180.0/M_PI));
	
		
	
	if(next_gps_waypoint != 7)
	{
		audibot_speed = Kp_speed*distance;// + Kd_speed*distance;
		
	}
	else
	{
		audibot_speed = audibot_speed_max;
	}
	
	if(audibot_speed<audibot_speed_min)
	{
		audibot_speed = audibot_speed_min;
	}
	if(audibot_speed>audibot_speed_max)
	{
		audibot_speed = audibot_speed_max;
	}
	if((next_gps_waypoint==3)||(next_gps_waypoint==4)||(next_gps_waypoint==5)||(next_gps_waypoint==6))
	{
		Kp_speed = 1.3;
		audibot_speed = Kp_speed*distance;
	}
	else
	{
		Kp_speed = 1.6;
	}
	
	steering_wheel_angle = Kp_angle*target_angle;
	
	
	if(distance<=1.0)
	{
		//ROS_INFO("\n\nReached at GPS waypoint %d\n\n",(next_gps_waypoint+1));
		next_gps_waypoint++;	
	}
	
	if(next_gps_waypoint>=n_waypoints)
	{
		audibot_speed = 0.0;
		audibot_speed_max = 0.0;
		
		steering_wheel_angle = 0.0;
		steering_wheel_angle_max = 0.0;
		ROS_INFO("\n\n\n*****	Target achieved..!	*****\n\n");
		while(true);
	}
	
	if((steering_wheel_angle>0.65)||(steering_wheel_angle<-0.65))
	{
		audibot_speed = audibot_speed_min;
	}
	
			
	if((next_gps_waypoint == 0)&& (distance<100)||(next_gps_waypoint==1))
	{
		audibot_speed = 16.0;
	}
	
	
	
	geometry_msgs::Twist speedSteerMsg;
	speedSteerMsg.linear.x = audibot_speed;
	speedSteerMsg.angular.z = steering_wheel_angle;
	
	pub_speed_steering.publish(speedSteerMsg);
	
}



void headingCallBackFunc(std_msgs::Float64 current_heading)
{
	psi = (90.0 - current_heading.data)*M_PI/180.0;
	//ROS_INFO("Current heading angle of audibot is: %f",psi*180.0/M_PI);
	
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_sim_project");
	ros::NodeHandle nh;
	
	// Set up topics and timer
	ros::Subscriber sub_fix = nh.subscribe("/audibot/gps/fix", 1, recvFix);
	
	// Subscribe to GPS heading and tie call back to compute heading error for next GPS waypoint reference 
	ros::Subscriber sub_heading_psi = nh.subscribe("/audibot/gps/heading",1,headingCallBackFunc);
	
	ros::Timer path_timer = nh.createTimer(ros::Duration(0.02), timerCallback);
	pub_path_points = nh.advertise<nav_msgs::Path>("/audibot/gps_path", 1);
	
	
	pub_speed_steering = nh.advertise<geometry_msgs::Twist>("/audibot/cmd_vel",1);
	
	
	// Advertise marker messages on markers topic
	pub_waypoints_marker  = nh.advertise<visualization_msgs::MarkerArray>("/audibot/gps_waypoints_marker", 1);
	
	
	// Read reference coordinates from the parameter server as lat/lon
	double ref_lat;
	double ref_lon;
	nh.param("/audibot/gps/ref_lat", ref_lat, 0.0);	// read lat from ros namespace
	nh.param("/audibot/gps/ref_lon", ref_lon, 0.0);	// read lon from ros namespace
	
	// Instantiate a LatLon instance with the reference geodetic coordinates
	LatLon ref_lat_lon(ref_lat, ref_lon, 0.0);
	
	// Pass the LatLon object instance to the UTMCoords constructor to convert
	// to UTM and store the resulting object in the global instance.
	ref_coords = UTMCoords(ref_lat_lon);
	
	// load lat lon co-ordinates published through yaml file
	double lat1,lat2,lat3,lat4,lat5,lat6,lat7,lat8;
	double lon1,lon2,lon3,lon4,lon5,lon6,lon7,lon8;
	
	// set Lat and Lon for all GPS waypoints from namspace
	nh.getParam("/audibot/waypoints/lat1",lat1);
	nh.getParam("/audibot/waypoints/lat2",lat2);
	nh.getParam("/audibot/waypoints/lat3",lat3);
	nh.getParam("/audibot/waypoints/lat4",lat4);
	nh.getParam("/audibot/waypoints/lat5",lat5);
	nh.getParam("/audibot/waypoints/lat6",lat6);
	nh.getParam("/audibot/waypoints/lat7",lat7);
	nh.getParam("/audibot/waypoints/lat8",lat8);
	
	
	nh.getParam("/audibot/waypoints/lon1",lon1);
	nh.getParam("/audibot/waypoints/lon2",lon2);
	nh.getParam("/audibot/waypoints/lon3",lon3);
	nh.getParam("/audibot/waypoints/lon4",lon4);
	nh.getParam("/audibot/waypoints/lon5",lon5);
	nh.getParam("/audibot/waypoints/lon6",lon6);
	nh.getParam("/audibot/waypoints/lon7",lon7);
	nh.getParam("/audibot/waypoints/lon8",lon8);
	
	// Set frame ID of the path for Rviz to render it properly
	gps_path.header.frame_id = "world";
	
	sensor_msgs::NavSatFix waypoint;
	
	
	double lat_array[n_waypoints] = {lat1,lat2,lat3,lat4,lat5,lat6,lat7,lat8};
	double lon_array[n_waypoints] = {lon1,lon2,lon3,lon4,lon5,lon6,lon7,lon8};
	
	
	for(static int i=0;i<n_waypoints;i++)
	{
		// Frame ID
		marker.header.frame_id = "/audibot/gps_waypoints_marker";
		
		// Action
		marker.action = visualization_msgs::Marker::ADD;
		
		// Type
		marker.type = visualization_msgs::Marker::CYLINDER ;
		
		// Scale of marker
		marker.scale.x = 2.0;
		marker.scale.y = 2.0;
		marker.scale.z = 0.1;
		
		// Color of marker
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;
		
		
		waypoint.header.frame_id ="world";
		waypoint.header.stamp = ros::Time::now();
		waypoint.latitude = lat_array[i];
		waypoint.longitude = lon_array[i];
		//recvFix(waypoint);
		
		UTMCoords waypoint_UTM(waypoint);
		relative_position_gps_waypoint[i] = waypoint_UTM - ref_coords;
		
		//pub_gps_waypoints.publish(waypoint);
		
		ROS_INFO("X pos of waypoint %d is: %f", i+1, relative_position_gps_waypoint[i].x());
		ROS_INFO("Y pos of waypoint %d is: %f", i+1, relative_position_gps_waypoint[i].y());
		ROS_INFO("Angle of %d waypoint with X axis(due east) in world frame is: %f ", i+1, \
						atan(relative_position_gps_waypoint[i].y()/relative_position_gps_waypoint[i].x()));
		//UTMCoords::UTMCoords ( lat_array[i], lon_array[i], 0.0, 17, 1);
		
		// Position
		marker.pose.position.x = relative_position_gps_waypoint[i].x();
		marker.pose.position.y = relative_position_gps_waypoint[i].y();
		marker.pose.position.z = 0.0;
		
		// Orientation
		marker.pose.orientation.w = 1.0;
		
		// Assign unique sequential ID to marker
		marker.id = i;
		
		// Add marker to MarkerArray message
		gps_waypoints_array.markers.push_back(marker);
	}	
		
	

	ros::spin();
}
