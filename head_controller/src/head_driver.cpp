#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

double des_pan_;

void panAngleCallback(const std_msgs::Float64& msg)
{
	des_pan_ = msg.data;
}

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "head_driver");
	ros::NodeHandle nh;
	tf::TransformBroadcaster br;
	ros::Subscriber angle_sub_ = nh.subscribe("/pan_angle", 1, &panAngleCallback);
	
	// TODO: constraints on pan velocity and acceleration
	double cmd_pan;
	double pan_vel_max, pan_vel_curr;
	double pan_acc_max, pan_acc_curr;
	
	// TODO: PD control
	double Kp, Kd;
	
	// We control a servo that determines the link between parent_frame and child_frame
	std::string parent_frame, child_frame;
	nh.param<std::string>("parent_tf_frame", parent_frame , "NULL");
	nh.param<std::string>("pan_tf_frame"   , child_frame  , "NULL");
	ROS_INFO("[head_driver] Broadcasting TF from \"%s\" to \"%s\"",
		parent_frame.c_str(), child_frame.c_str());
	
	// Control loop rate
	double loop_rate_dbl;
	nh.param("loop_rate", loop_rate_dbl, 10.0);
	ros::Rate loop_rate = ros::Rate(loop_rate_dbl);
	ROS_INFO("[head_driver] Loop rate is %.2fHz", loop_rate_dbl);
	
	while( ros::ok() )
	{
		// TODO: PD control loop on the desired angle
		cmd_pan = des_pan_;
		
		// TODO: Send the commanded angle to the servo
		ROS_INFO_THROTTLE(2, "[head_driver] Commanded angle %.2f", cmd_pan);
	
		// Create and publish a transform. No translation, one degree of rotation (pan).
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
		transform.setRotation( tf::Quaternion(cmd_pan, 0.0, 0.0) );
		br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame );
	
		// Allow callbacks to occur, and sleep to enforce the desired rate.
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("[head_driver] Shutdown now.");
	return(0);
}
