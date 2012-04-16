#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using std::string;
using ros::Duration;
using geometry_msgs::Twist;
using geometry_msgs::PoseStamped;
using geometry_msgs::PoseWithCovarianceStamped;

const double pi = 3.1415926;

Twist zeroVelocity()
{
	Twist t;
	t.linear.x  = 0; t.linear.y  = 0; t.linear.z  = 0;
	t.angular.x = 0; t.angular.y = 0; t.angular.z = 0;
	return t;
}

// Takes time (seconds), amplitude (rad) and top speed (rad/sec) and generates a sine wave spin velocity
// angle = A    sin(w*t)
// speed = A*w  cos(w*t)
//
// A*w = Vmax
// Using amplitude/2 because we want amax - amin
Twist sinInPlace(double elapsed_secs, double amplitude, double vmax)
{
	Twist t      = zeroVelocity();
	double omega = vmax/(amplitude/2);
	t.angular.z  = vmax * cos(omega * elapsed_secs);
	
	return t;
}

Twist spin(double speed)
{
	Twist t     = zeroVelocity();
	t.angular.z = speed;
	
	return t;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_test_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	tf::TransformListener tfl;
	
	if( argc != 2 ) {
		ROS_ERROR("Call this node with one argument");
		return 1;
	}
	
	// Advertise some topics
	ros::Publisher head_pos_pub = nh.advertise<PoseWithCovarianceStamped>("/person_tracker/goal_with_covariance", 1);
	ros::Publisher head_pos_pub_no_cov = nh.advertise<PoseStamped>("/person_tracker/goal", 1);
	ros::Publisher cmd_vel_pub  = nh.advertise<Twist>("/cmd_vel", 1);
	
	// Get some parameters
	double amplitude, vmax, loop_rate_temp;
	string s;
	bool pub_person;
	pnh.param<std::string>("mode", s, "none");
	pnh.param("amplitude" , amplitude     , 1.0 );
	pnh.param("vmax"      , vmax          , 1.0 );
	pnh.param("loop_rate" , loop_rate_temp, 25.0);
	pnh.param("pub_person", pub_person    , false);
	ros::Rate loop_rate(loop_rate_temp);
	
	ROS_INFO_STREAM(boost::format("[motion_tests] Amplitude %.2f rad, top speed %.2f rad/sec") % amplitude %vmax );
	ROS_INFO_STREAM("[motion_tests] Loop rate is " << loop_rate_temp << "Hz");
	
	// Give a countdown to start
	ros::Duration step(1.0);
	for( Duration t(3.0); t > Duration(0); t -= step ) {
		ROS_INFO_STREAM("Starting in " << (t.toSec()));
		step.sleep();
	}
	
	ros::Time t0 = ros::Time::now();
	Twist     cmd_vel;
	double    elapsed_secs;
	
	// A fake person measurement in front of the robot
	PoseStamped pose;
	pose.header.frame_id  = "base_link";
	pose.header.stamp     = t0 - Duration(0.2);
	pose.pose.position.x  = 2.0;
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	try {
		tfl.transformPose("odom", pose, pose);
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	
	PoseWithCovarianceStamped pose_cov;
	pose_cov.header    = pose.header;
	pose_cov.pose.pose = pose.pose;

	while(ros::ok())
	{
		elapsed_secs      = (ros::Time::now() - t0).toSec();
		pose.header.stamp = ros::Time::now();
		pose_cov.header   = pose.header;
		
		if( s ==  "sin_in_place" ) {
			cmd_vel = sinInPlace(elapsed_secs, amplitude, vmax);
		}
		else if( s ==  "spin" ) {
			cmd_vel = spin(vmax);
		}
		else {
			ROS_ERROR_STREAM("Unknown option \"" << s << "\"");
			return 1;
		}
		cmd_vel_pub.publish(cmd_vel);
		
		ROS_INFO("%5.2f: lin = %5.2f ang = %5.2f", elapsed_secs, cmd_vel.linear.x, cmd_vel.angular.z);
		
		// Tell the head where to point
		if( pub_person )
		{
			head_pos_pub.publish(pose_cov);
			head_pos_pub_no_cov.publish(pose);
		}
		
		loop_rate.sleep();
	}
	
	return 0;
}
