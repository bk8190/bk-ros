#include <ros/ros.h>
#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>
#include <body_msgs/Skeletons.h>
#include <body_msgs/Skeleton.h>
#include <body_msgs/SkeletonJoint.h>

#include <stdlib.h>
#include <stdio.h>

#include <sound_play/sound_play.h>
#include "pt_sounds.cpp"

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
		
		geometry_msgs::Point person_pos_;
		bool found_person_;
};

PersonTracker::PersonTracker(int var) : 
	nh_("person_tracker")
{
	skeleton_sub_ = nh_.subscribe("/skeletons", 1, &PersonTracker::skeletonCB, this );
	found_person_ = false;
	ROS_INFO("Person tracker constructor finished");
}

bool PersonTracker::hasPerson()
{
	return(found_person_);
}

geometry_msgs::Point PersonTracker::getPersonPosition()
{
	return(person_pos_);
}
		
void PersonTracker::skeletonCB(const body_msgs::Skeletons& skel_msg)
{
	ROS_INFO("Person tracker got data, %lu skeletons.", skel_msg.skeletons.size());

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
			person_pos_ = torso.position;
			found_person_ = true;
			break;
		}
	}
}

void clamp( double& x, const double low, const double high )
{
	if( x > high )
		x = high;
	if( x < low )
		x = low;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "person_tracker");
	PersonTracker pt(0);
	ros::Publisher cmd_vel_pub = pt.nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	PTSounds::PTSoundPlayer sound_player(pt.nh_);
	ros::Duration(1.0).sleep();
	ROS_INFO("Initialization done.");
	
	
	sound_player.update();
	
	ros::Rate loop_rate(20);
	geometry_msgs::Point person_pos;
	geometry_msgs::Twist cmd_vel;
	
	while( ros::ok() )
	{
		if( pt.hasPerson() )
		{
			person_pos = pt.getPersonPosition();
			ROS_INFO("Found a person at x=%.2f\ty=%.2f\tz=%.2f\t", person_pos.x, person_pos.y, person_pos.z);
			sound_player.setState(PTSounds::state_tracking);
			
			// Linear feedback from position of person relative to camera
			cmd_vel.angular.z = person_pos.x;
			
			if( std::abs(person_pos.z - 1.50) > 0.1 ) {
				cmd_vel.linear.x = (person_pos.z-1.50)*1.0 ;
			}
		}
		else
		{
			sound_player.setState(PTSounds::state_searching);
			cmd_vel.angular.z = 0;
			cmd_vel.linear.x = 0;
		}
		
		// Ensure the robot doesn't run off
		clamp(cmd_vel.angular.z, -1.0, 1.0);
		clamp(cmd_vel.linear.x , -0.3, 0.3);
		ROS_INFO("CMD_VEL: x=%.2f\tz=%.2f", cmd_vel.linear.x, cmd_vel.angular.z);
		
		// Publish command velocity, 
		cmd_vel_pub.publish(cmd_vel);
		sound_player.update();
  	ros::spinOnce();
		loop_rate.sleep();
	}
	return(0);
}
