#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using std::string;

geometry_msgs::PoseStamped latest_person_;
geometry_msgs::Twist       latest_head_speed_;
nav_msgs::Odometry         latest_odom_;
bool                       has_lock_;

void personPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	latest_person_.header = msg->header;
	latest_person_.pose   = msg->pose.pose;
	ROS_DEBUG("[kinect_evaluation] Got person");
}

void headSpeedCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	latest_head_speed_ = *msg;
	ROS_DEBUG("[kinect_evaluation] Got head speed");
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	latest_odom_ = *msg;
	ROS_DEBUG("[kinect_evaluation] Got odom");
}

void hasLockCallback(const std_msgs::Float64::ConstPtr& msg)
{
	has_lock_ = (msg->data) > 0.5;
	ROS_DEBUG_STREAM("[kinect_evaluation] Got lock = " << (has_lock_ ? "True" : "False") );
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect_evaluation");
	ros::NodeHandle nh_;
	ros::NodeHandle pnh("~");
	
	//tf::TransformListener tfl;
	
	double loop_rate_temp;
	pnh.param("loop_rate", loop_rate_temp, 5.0);
	ros::Rate loop_rate(loop_rate_temp);
	ROS_INFO_STREAM("[kinect_evaluation] Loop rate is " << loop_rate_temp << "Hz");
	
	// Subscribe to stuff
	ros::Subscriber odom_sub       = nh_.subscribe("/odom"                        , 1, &odomCallback);
	ros::Subscriber ni_lock_sub    = nh_.subscribe("/bk_skeletal_tracker/has_lock", 1, &hasLockCallback);
	ros::Subscriber person_pos_sub = nh_.subscribe("/goal_with_covariance"        , 1, &personPosCallback);
	ros::Subscriber head_speed_sub = nh_.subscribe("/head_controller/head_driver/head_speed", 1, &headSpeedCallback);
	
	ros::Duration time_with_lock(0.0001);
	ros::Duration total_time    (0.0001);
	
	double doody_cycle;
	bool   prev_has_lock = has_lock_;
	
	std::stringstream ss;
	ss << "\ncondition,time,speed_lin,speed_ang,head_speed\n";
	
	loop_rate.sleep();
	while( ros::ok() )
	{
		total_time += loop_rate.expectedCycleTime();
		if( has_lock_ ) {
			time_with_lock += loop_rate.expectedCycleTime();
		}
	
		doody_cycle = 100.0 * (time_with_lock.toSec() / total_time.toSec());
		ROS_INFO_STREAM(boost::format("Running for %.2fsec, Duty cycle = %2f%%")
		  % (total_time.toSec()) % doody_cycle );
		
		
		// If we just acquired or dropped the target, record the conditions when it happened
		if( prev_has_lock != has_lock_ )
		{
			ss << (has_lock_ ? "add" : "drop");
			ss << boost::format("%.3f,") % total_time.toSec();
			ss << boost::format("%.3f,") % latest_odom_.twist.twist.linear.x;
			ss << boost::format("%.3f,") % latest_odom_.twist.twist.angular.z;
			ss << boost::format("%.3f,") % latest_head_speed_.angular.z;
			ss << "\n";
			
			prev_has_lock = has_lock_;
		}
		
		string s = ss.str();
		ROS_INFO_STREAM(s);
		
		loop_rate.sleep(); // Sleep to enforce loop rate
		ros::spinOnce();   //  Allow callbacks to occur
		
		if( loop_rate.cycleTime() > loop_rate.expectedCycleTime() ) {
			ROS_WARN_STREAM(boost::format("[kinect_evaluation] Missed update rate of %.3f sec, took %.3f sec")
			  % (loop_rate.expectedCycleTime().toSec()) % (loop_rate.cycleTime().toSec()) );
		}
	}
}
