#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>

using std::string;
using geometry_msgs::Twist;

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
	
	t.angular.z = amplitude*pi/180 * sin(elapsed_secs * 2*pi/period);
	
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
	
	
	ros::Publisher cmd_vel_pub = nh.advertise<Twist>("/cmd_vel", 1);
	ros::Rate      loop_rate(25);
	Twist          cmd_vel;
	ros::Time      t0 = ros::Time::now();
	double         elapsed_secs;
	
	while(ros::ok())
	{
		elapsed_secs = (ros::Time::now() - t0).toSec();
		
		if( s ==  "sin_in_place" ) {
			cmd_vel = sinInPlace(elapsed_secs, 50, 5.0);
		}
		else {
			ROS_ERROR_STREAM("Unknown option \"" << s << "\"");
			return 1;
		}
		
		ROS_INFO("%5.2f: lin = %5.2f ang = %5.2f", elapsed_secs, cmd_vel.linear.x, cmd_vel.angular.z);
	
		cmd_vel_pub.publish(cmd_vel);
		loop_rate.sleep();
	}
	
	return(0);
}
