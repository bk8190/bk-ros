#ifndef PERSONTRACKER_H
#define PERSONTRACKER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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
		tf::TransformListener tf_; 
		PTSounds::PTSoundPlayer sound_player_;
		bool found_person_;
		ros::Time     last_detect_;
		
		ros::Subscriber skeleton_sub_;
		ros::Publisher  goal_pub_;
		ros::Timer      compute_loop_timer_;
		PoseStamped     person_pos_;
		PoseStamped     last_pub_pose_;
		
		double          loop_rate_;
		double goal_hysteresis_;
		ros::Duration detect_timeout_;
		
		void skeletonCB(const body_msgs::Skeletons& skel_msg);
		void computeStateLoop(const ros::TimerEvent& event);
		bool poseToGlobalFrame(const PoseStamped& pose_msg, PoseStamped& transformed);
		bool getFirstGoodJoint(const body_msgs::Skeleton& skel, body_msgs::SkeletonJoint& body_part, string& body_part_name);

};

};//namespace
#endif
