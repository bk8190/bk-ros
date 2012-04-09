#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using std::string;
using geometry_msgs::Twist;
using geometry_msgs::PoseWithCovarianceStamped;

const double pi = 3.1415926;

Twist zeroVelocity()
{
	Twist t;
	t.linear.x  = 0;
	t.linear.y  = 0;
	t.linear.z  = 0;
	t.angular.x = 0;
	t.angular.y = 0;
	t.angular.z = 0;
	
	return t;
}

// Takes time (seconds), amplitude (degrees) and period (seconds) and generates a sine wave spin velocity
Twist sinInPlace(double elapsed_secs, double amplitude, double period)
{
	Twist t = zeroVelocity();
	
	t.angular.z = amplitude * sin(elapsed_secs * 2*pi/period);
	
	return t;
}

Twist spin(double speed)
{
	Twist t = zeroVelocity();
	
	t.angular.z = speed;
	
	return t;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "motion_test_node");
	ros::NodeHandle nh;
	
	if( argc != 2 ) {
		ROS_ERROR("Call this node with one argument");
		return 1;
	}
	
	string s = argv[1];
	
	PoseWithCovarianceStamped p;
	p.header.frame_id = "odom";
	
	ros::Publisher head_pos_pub = nh.advertise<PoseWithCovarianceStamped>("/person_tracker/goal_with_covariance", 1);
	
	ros::Publisher cmd_vel_pub  = nh.advertise<Twist>("/cmd_vel", 1);
	ros::Rate      loop_rate(25);
	Twist          cmd_vel;
	ros::Time      t0 = ros::Time::now();
	double         elapsed_secs;
	
	for( int i=3; i>0; i-- )
	{
		ROS_INFO_STREAM("Starting in " << i);
		ros::Duration(1.0).sleep();
	}
	
	while(ros::ok())
	{
		elapsed_secs = (ros::Time::now() - t0).toSec();
		p.header.stamp = ros::Time::now();
		
		if( s ==  "sin_in_place" ) {
			cmd_vel = sinInPlace(elapsed_secs, .80, 5.0);
			//cmd_vel = sinInPlace(elapsed_secs, .40, 10.0);
		}
		else if( s ==  "spin" ) {
			cmd_vel = spin(.60);
		}
		else {
			ROS_ERROR_STREAM("Unknown option \"" << s << "\"");
			return 1;
		}
		
		ROS_INFO("%5.2f: lin = %5.2f ang = %5.2f", elapsed_secs, cmd_vel.linear.x, cmd_vel.angular.z);
	
		cmd_vel_pub.publish(cmd_vel);
		head_pos_pub.publish(p); // Tell the head where to point
		loop_rate.sleep();
	}
	
	return(0);
}
