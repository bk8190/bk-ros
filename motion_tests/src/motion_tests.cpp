#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>

using std::string;
using geometry_msgs::Twist;

ros::Publisher cmd_vel_pub_
ros::Rate      loop_rate_;

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

void sinInPlace(Twist& t, double elapsed_secs)
{

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
	ros::Duration  elapsed_secs;
	
	while(ros::ok())
	{
		cmd_vel         = zeroVelocity();
		elapsed_secs = (ros::Time::now() - t0).toSec();
		
		switch(s)
		{
			case "sin_in_place":
				cmd_vel = sinInPlace(elapsed_secs)
			break;
		
			default:
				ROS_ERROR_STREAM("Unknown option \"" << s << "\"");
				return 1;
		}
	
	
		cmd_vel_pub.publish(cmd_vel);
		loop_rate_.sleep();
	}
	
	return(0);
}
