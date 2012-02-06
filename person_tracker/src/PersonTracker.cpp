#include <ros/ros.h>
#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>
#include <body_msgs/Skeletons.h>
#include <body_msgs/Skeleton.h>
#include <body_msgs/SkeletonJoint.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

using std::cout;
using std::endl;
using std::string;

class PersonTracker
{
	public:
		int var_;
		ros::NodeHandle nh_;

		ros::Subscriber skeleton_sub_;
	
		bool hasPerson();
		geometry_msgs::Point getPersonPosition();
	
		PersonTracker(int var);
		~PersonTracker()
		{
		}

	private:
		void skeletonCB(const body_msgs::Skeletons& skel_msg);
		
		geometry_msgs::Point person_position_;
		bool found_person_;
};

PersonTracker::PersonTracker(int var) : 
	nh_("person_tracker")
{
	skeleton_sub_ = nh_.subscribe("/skeletons", 1, &PersonTracker::skeletonCB, this );
	ROS_INFO("Person tracker constructor finished");
}

bool PersonTracker::hasPerson()
{
	return(found_person_);
}

geometry_msgs::Point PersonTracker::getPersonPosition()
{
	return(person_position_);
}
		
void PersonTracker::skeletonCB(const body_msgs::Skeletons& skel_msg)
{
	ROS_INFO("Person tracker got data, %d skeletons.", skel_msg.skeletons.size());

	body_msgs::SkeletonJoint torso;
	found_person_ = false;
	
	// Find the first torso position
	for( int i=0; i<skel_msg.skeletons.size(); i++ )
	{
		// Get the torso position and the player's ID
		torso = skel_msg.skeletons.at(i).torso;
		
		ROS_INFO("Player %d has confidence %f", skel_msg.skeletons.at(i).playerid, torso.confidence);
		
		// If the confidence is high enough (>50%) save the location
		if( torso.confidence > 0.5 )
		{
			person_position_ = torso.position;
			found_person_ = true;
			break;
		}
	}
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "person_tracker");
	PersonTracker pt(0);
	ros::Publisher cmd_vel_pub = pt.nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		
	ROS_INFO("Initialization done.");
	
	ros::Rate loop_rate(20);
	geometry_msgs::Point person_position;
	geometry_msgs::Twist cmd_vel;
	
	while( ros::ok() )
	{
		cmd_vel.angular.z = 0;
		cmd_vel.linear.x = 0;
		
		if( pt.hasPerson() )
		{
			person_position = pt.getPersonPosition();
			
			ROS_INFO("Found a person at x=%.2f\ty=%.2f\tz=%.2f\t", person_position.x, person_position.y, person_position.z);

			cmd_vel.angular.z = person_position.x;
			
			if( fabs(person_position.z - 1.50) > 0.1 )
			{
				cmd_vel.linear.x = (person_position.z-1.50)*1.0 ;
			}
		}
		
		if(cmd_vel.angular.z > 1.0)
			cmd_vel.angular.z = 1.0;
		if(cmd_vel.angular.z < -1.0)
			cmd_vel.angular.z = -1.0;
		
		
		if(cmd_vel.angular.x > .8)
			cmd_vel.angular.x = .8;
		if(cmd_vel.angular.x < -.8)
			cmd_vel.angular.x = -.8;
		
		
		ROS_INFO("CMD_VEL: x=%.2f\tz=%.2f", cmd_vel.linear.x, cmd_vel.angular.z);
		
		// DEBUG
		//cmd_vel.angular.z = .5;
		
		cmd_vel_pub.publish(cmd_vel);
  	ros::spinOnce();
		loop_rate.sleep();
	}
	return(0);
}
