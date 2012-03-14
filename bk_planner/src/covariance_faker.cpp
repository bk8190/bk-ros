// Simple node that subscribes to a pose and publishes the same pose with faked covariance.
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pose_cov_pub_;

void
poseCB(const geometry_msgs::PoseStamped::ConstPtr& pose_ptr)
{
	geometry_msgs::PoseWithCovarianceStamped p;
	p.header    = pose_ptr->header;
	p.pose.pose = pose_ptr->pose;
	
	// The covariance is made of lies
	for( int i=0; i<36; i++ ) {
		p.pose.covariance[i] = 0.0;
	}

	pose_cov_pub_.publish(p);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "covariance_faker");
	ros::NodeHandle nh;
	ROS_INFO("[covariance_faker] Initialized");
	
	ros::Subscriber pose_sub = nh.subscribe("pose", 1, &poseCB);
	pose_cov_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 1);
	
	// The callback takes care of the rest
	ros::spin();
	return 0;
}
