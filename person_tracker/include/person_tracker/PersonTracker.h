#ifndef PERSONTRACKER_H
#define PERSONTRACKER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <portal_sounds/PTSounds.h>

#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
using geometry_msgs::PoseWithCovarianceStamped;

class PersonTracker
{
	public:
		
		bool hasPerson();
		ros::Duration timeSinceLastDetect();
		PoseWithCovarianceStamped getPersonPosition();
	
		PersonTracker(string name);
		~PersonTracker()
		{
		}

	private:
		ros::NodeHandle         nh_;
		tf::TransformListener   tf_; 
		PTSounds::PTSoundPlayer sound_player_;
		ros::Time               last_detect_;
		bool                    new_goal_;
		
		ros::Subscriber skeleton_sub_;
		ros::Publisher  goal_pub_;
		ros::Publisher  goal_cov_pub_;
		ros::Timer      compute_loop_timer_;
		
		PoseWithCovarianceStamped person_pos_;
		PoseWithCovarianceStamped last_pub_pos_;
		
		ros::Duration detect_timeout_;
		double alpha_;
		double loop_rate_;
		double goal_hysteresis_;
		double pos_var_start_;
		double var_increase_rate_;
		double max_var_;
		
		void skeletonCB(const body_msgs::Skeletons& skel_msg);
		void computeStateLoop(const ros::TimerEvent& event);
		PoseStamped poseToGlobalFrame(const PoseStamped& pose_msg);
		bool poseToGlobalFrame(PoseStamped pose_msg, PoseStamped& transformed);
		bool poseToGlobalFrame(const PoseWithCovarianceStamped& pose_msg, PoseWithCovarianceStamped& transformed);
		bool getFirstGoodJoint(const body_msgs::Skeleton& skel, double confidence_thresh, body_msgs::SkeletonJoint& body_part, string& body_part_name);

};

};//namespace
#endif
