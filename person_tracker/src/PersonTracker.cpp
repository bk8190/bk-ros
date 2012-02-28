#include <person_tracker/PersonTracker.h>

namespace PersonTracker {

PersonTracker::PersonTracker(string name) :
	nh_           (name),
	tf_           (ros::Duration(10)),
	sound_player_ (nh_, "sounds"),
	last_detect_  (ros::Time(0))
{
	nh_.param("loop_rate", loop_rate_, 2.0); // default 2Hz
	double temp;
	nh_.param("detect_timeout", temp, 3.0);
	detect_timeout_ = ros::Duration(temp);
	
	ROS_INFO("[person tracker] Detect timeout  is %.2f", detect_timeout_.toSec());
	
	skeleton_sub_ = nh_.subscribe("/skeletons", 1, &PersonTracker::skeletonCB, this );
	goal_pub_     = nh_.advertise<PoseStamped>("goal", 1);
	
	
	//Setup a callback to occur at a regular rate
  double dt = 1.0/loop_rate_;
	ROS_INFO("[person tracker] Loop rate is %.2fHz (%.2f seconds)", loop_rate_, dt);
	compute_loop_timer_ = nh_.createTimer(ros::Duration(dt), boost::bind(&PersonTracker::computeStateLoop, this, _1));
}

bool
PersonTracker::hasPerson()
{
	ros::Duration time_since_last = ros::Time::now() - last_detect_;
	ROS_INFO("[person_tracker] Last detect %.2fs ago", time_since_last.toSec());
	return(time_since_last < detect_timeout_);
}

PoseStamped
PersonTracker::getPersonPosition()
{
	return(person_pos_);
}

/*
bool
PersonTracker::poseToGlobalFrame(const PoseStamped& pose_msg, PoseStamped& transformed)
{
	std::string global_frame = "odom";
	tf::Stamped<tf::Pose> goal_pose, global_pose;
	poseStampedMsgToTF(pose_msg, goal_pose);

	//just get the latest available transform
	goal_pose.stamp_ = ros::Time();

	try {
		tf_.transformPose(global_frame, goal_pose, global_pose);
	}
	catch(tf::TransformException& ex) {
		ROS_WARN("[person_tracker] Failed to transform the goal pose from %s into the %s frame: %s",
		goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return false;
	}

	PoseStamped global_pose_msg;
	tf::poseStampedTFToMsg(global_pose, global_pose_msg);
	transformed = global_pose_msg;
	return true;
}*/

// Searches through the joints in the skeleton, and returns the first one with good confidence
bool
PersonTracker::getFirstGoodJoint(const body_msgs::Skeleton& skel, body_msgs::SkeletonJoint& body_part, string& body_part_name)
{
	double confidence_thresh = 0.5;

	if( skel.head.confidence > confidence_thresh ) {
		body_part = skel.head;
		body_part_name = "head";
		return true;
	}
	
	if( skel.neck.confidence > confidence_thresh ) {
		body_part = skel.neck;
		body_part_name = "neck";
		return true;
	}
	
	if( skel.right_shoulder.confidence > confidence_thresh ) {
		body_part = skel.right_shoulder;
		body_part_name = "right_shoulder";
		return true;
	}
	
	if( skel.left_shoulder.confidence > confidence_thresh ) {
		body_part = skel.left_shoulder;
		body_part_name = "left_shoulder";
		return true;
	}
	
	if( skel.right_elbow.confidence > confidence_thresh ) {
		body_part = skel.right_elbow;
		body_part_name = "right_elbow";
		return true;
	}
	
	if( skel.left_elbow.confidence > confidence_thresh ) {
		body_part = skel.left_elbow;
		body_part_name = "left_elbow";
		return true;
	}
	
	if( skel.torso.confidence > confidence_thresh ) {
		body_part = skel.torso;
		body_part_name = "torso";
		return true;
	}
	
	if( skel.right_hip.confidence > confidence_thresh ) {
		body_part = skel.right_hip;
		body_part_name = "right_hip";
		return true;
	}
	
	if( skel.left_hip.confidence > confidence_thresh ) {
		body_part = skel.left_hip;
		body_part_name = "left_hip";
		return true;
	}
	
	return false;

}

void
PersonTracker::skeletonCB(const body_msgs::Skeletons& skel_msg)
{
	ROS_INFO_THROTTLE(5,"[person tracker] Got data, %lu skeletons.", skel_msg.skeletons.size());

	body_msgs::SkeletonJoint body_part;
	string                   body_part_name;
	
	// Find the first trackable person
	for( unsigned int i=0; i<skel_msg.skeletons.size(); i++ )
	{
		// If the person has a trackable joint, save the location
		if( getFirstGoodJoint(skel_msg.skeletons.at(i), body_part, body_part_name) )
		{
			ROS_INFO_THROTTLE(1,"[person tracker] Player %d's %s has confidence %f", skel_msg.skeletons.at(i).playerid, body_part_name.c_str(), body_part.confidence);
			
			person_pos_.pose.position    = body_part.position;
			
			// TODO: Fix this dirty hack.  I am mirroring the image over its vertical axis which should be done by a transform
			person_pos_.pose.position.x = body_part.position.x * -1.0;
			
			person_pos_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			//ROS_INFO_THROTTLE(1,"[person tracker] Skeletons' frame ID is %s", skel_msg.header.frame_id.c_str());
			person_pos_.header.frame_id  = skel_msg.header.frame_id;
			person_pos_.header.stamp     = skel_msg.header.stamp;
			
			last_detect_ = ros::Time::now();
			
			break;
		}
	}
}

void
PersonTracker::computeStateLoop(const ros::TimerEvent& event) {
	ROS_DEBUG("[person tracker] Last callback took %f seconds", event.profile.last_duration.toSec());

	// Found a person -> publish the position
	if( hasPerson() )
	{
		sound_player_.setState(PTSounds::state_tracking);
		
		PoseStamped pub = person_pos_;
		goal_pub_.publish(pub);
		ROS_INFO("[person tracker] Published: person at x=%.2f\ty=%.2f\tz=%.2f\t", pub.pose.position.x, pub.pose.position.y, pub.pose.position.z);
	}
	else {
		ROS_INFO("[person tracker] No target");
		sound_player_.setState(PTSounds::state_searching);
	}
	
	sound_player_.update();
}

};//namespace

int main (int argc, char** argv)
{
  // Initialize ROS and create a PersonTracker object
  ros::init (argc, argv, "person_tracker_node");
	PersonTracker::PersonTracker pt("person_tracker");
	
	ros::spin();
	return(0);
}
