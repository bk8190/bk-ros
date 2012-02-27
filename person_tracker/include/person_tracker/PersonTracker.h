#ifndef PERSONTRACKER_H
#define PERSONTRACKER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <person_tracker/PTSounds.h>

#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>
#include <body_msgs/Skeletons.h>
#include <body_msgs/Skeleton.h>
#include <body_msgs/SkeletonJoint.h>

#include <stdlib.h>
#include <stdio.h>


namespace PersonTracker {

using std::string;
using geometry_msgs::PoseStamped;

class PersonTracker
{
	public:
		
		bool hasPerson();
		PoseStamped getPersonPosition();
	
		PersonTracker(string name);
		~PersonTracker()
		{
		}

	private:
		ros::NodeHandle nh_;
		PTSounds::PTSoundPlayer sound_player_;
		bool found_person_;
		PoseStamped last_pub_pose_;
		double goal_hysteresis_;
		ros::Subscriber skeleton_sub_;
		ros::Publisher  goal_pub_;
		ros::Timer      compute_loop_timer_;
		double          loop_rate_;
		
		void skeletonCB(const body_msgs::Skeletons& skel_msg);
		void computeStateLoop(const ros::TimerEvent& event);
		
		geometry_msgs::PoseStamped person_pos_;
};

};//namespace
#endif
