#include <ros/ros.h>
#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <body_msgs/Skeletons.h>
#include <body_msgs/Skeleton.h>
#include <body_msgs/SkeletonJoint.h>

#include <stdlib.h>
#include <stdio.h>

using std::cout;
using std::endl;
using std::string;

class PersonTracker
{
	public:
		int var_;
		ros::NodeHandle nh_;

		ros::Subscriber skeleton_sub_;
	
		PersonTracker(int var);
		~PersonTracker()
		{
		}

	private:
		void skeletonCB(const body_msgs::Skeletons& skel_msg);
};

PersonTracker::PersonTracker(int var) : 
	nh_("person_tracker")
{
	skeleton_sub_ = nh_.subscribe("/skeletons", 1, &PersonTracker::skeletonCB, this );
	ROS_INFO("Person tracker constructor finished");
}

void PersonTracker::skeletonCB(const body_msgs::Skeletons& skel_msg)
{
	ROS_INFO("Person tracker got data, %d skeletons.", skel_msg.skeletons.size());

	geometry_msgs::Point position;
	bool found_person = false;
	
	if( skel_msg.skeletons.size() > 0 )
	{
		// Get the torso position
		body_msgs::SkeletonJoint torso = skel_msg.skeletons.at(0).torso;

		if( torso.confidence > 0.5 )
		{
			position = torso.position;
			found_person = true;
		}
	}
	
	
	if( found_person )
	{
		ROS_INFO("Found a person at x=%.2f\ty=%.2f\tz=%.2f\t", position.x, position.y, position.z);
		
	}
	
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "person_tracker");
	PersonTracker pt(0);
	ROS_INFO("Initialization done.");
  ros::spin();
	return(0);
}
