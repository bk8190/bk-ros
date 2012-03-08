#include <person_tracker/PersonTracker.h>

namespace PersonTracker {

// Strips covariance from a pose
PoseStamped stripCovariance(const PoseWithCovarianceStamped& p)
{
	PoseStamped p2;
	p2.header = p.header;
	p2.pose   = p.pose.pose;
	
	return p2;
}

// Sets the x, y, and z variance, zeros all other terms
void setVariance( PoseWithCovarianceStamped& p, double var )
{
	// Initialize covariance matrix (x,y,z,xrot,yrot,zrot) to all zeros
	for(int j=0; j<36; j++) {
		p.pose.covariance[j] = 0.0;
	}
	
	// Set x,y,z self-variances to nonzero
	p.pose.covariance[0+6*0] = var;
	p.pose.covariance[1+6*1] = var;
	p.pose.covariance[2+6*2] = var;
}

// Returns variance along the x direction
double getVariance(const PoseWithCovarianceStamped& p)
{
	return p.pose.covariance[0];
}

PersonTracker::PersonTracker(string name) :
	nh_           (name),
	tf_           (ros::Duration(15)),
	sound_player_ (nh_, "sounds"),
	last_detect_  (ros::Time(0)),
	new_goal_     (false)
{
	nh_.param("loop_rate", loop_rate_, 2.0); // default 2Hz
	double temp;
	nh_.param("detect_timeout", temp, 3.0);
	detect_timeout_ = ros::Duration(temp);
	
	ROS_INFO("[person tracker] Detect timeout  is %.2f", detect_timeout_.toSec());
	
	skeleton_sub_ = nh_.subscribe("/skeletons", 1, &PersonTracker::skeletonCB, this );
	goal_pub_     = nh_.advertise<PoseStamped>("goal", 1);
	goal_cov_pub_ = nh_.advertise<PoseWithCovarianceStamped>("goal_with_covariance", 1);
	
	person_pos_.pose.pose.position.x = 0.0;
	person_pos_.pose.pose.position.y = 0.0;
	person_pos_.pose.pose.position.z = 0.0;
	person_pos_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	person_pos_.header.frame_id = "/base_link";
	
	//Set a callback to occur at a regular rate
  double dt = 1.0/loop_rate_;
	ROS_INFO("[person tracker] Loop rate is %.2fHz (%.2f seconds)", loop_rate_, dt);
	compute_loop_timer_ = nh_.createTimer(ros::Duration(dt), boost::bind(&PersonTracker::computeStateLoop, this, _1));
	
	pos_var_start_     = 0.1;
	var_increase_rate_ = 0.5;
	max_var_           = 5.0;
}

ros::Duration PersonTracker::timeSinceLastDetect()
{
	return ros::Time::now() - last_detect_;
}

bool
PersonTracker::hasPerson()
{
	return timeSinceLastDetect() < detect_timeout_;
}

PoseWithCovarianceStamped
PersonTracker::getPersonPosition()
{
	return person_pos_;
}

// Searches through the joints in the skeleton, and returns the first one with good confidence
bool
PersonTracker::getFirstGoodJoint(const body_msgs::Skeleton& skel, double confidence_thresh, body_msgs::SkeletonJoint& body_part, string& body_part_name)
{
	if( skel.head.confidence >= confidence_thresh ) {
		body_part = skel.head;
		body_part_name = "head";
		return true;
	}
	
	if( skel.neck.confidence >= confidence_thresh ) {
		body_part = skel.neck;
		body_part_name = "neck";
		return true;
	}
	
	if( skel.right_shoulder.confidence >= confidence_thresh ) {
		body_part = skel.right_shoulder;
		body_part_name = "right_shoulder";
		return true;
	}
	
	if( skel.left_shoulder.confidence >= confidence_thresh ) {
		body_part = skel.left_shoulder;
		body_part_name = "left_shoulder";
		return true;
	}
	
	if( skel.right_elbow.confidence >= confidence_thresh ) {
		body_part = skel.right_elbow;
		body_part_name = "right_elbow";
		return true;
	}
	
	if( skel.left_elbow.confidence >= confidence_thresh ) {
		body_part = skel.left_elbow;
		body_part_name = "left_elbow";
		return true;
	}
	
	if( skel.torso.confidence >= confidence_thresh ) {
		body_part = skel.torso;
		body_part_name = "torso";
		return true;
	}
	
	if( skel.right_hip.confidence >= confidence_thresh ) {
		body_part = skel.right_hip;
		body_part_name = "right_hip";
		return true;
	}
	
	if( skel.left_hip.confidence >= confidence_thresh ) {
		body_part = skel.left_hip;
		body_part_name = "left_hip";
		return true;
	}
	
	return false;

}

bool
PersonTracker::poseToGlobalFrame(const PoseWithCovarianceStamped& pose_msg, PoseWithCovarianceStamped& transformed)
{
	PoseStamped p2;
	bool success = poseToGlobalFrame(stripCovariance(pose_msg), p2);
	
	transformed.header    = p2.header;
	transformed.pose.pose = p2.pose;
	transformed.pose.covariance = pose_msg.pose.covariance;
	
	return success;
}

bool
PersonTracker::poseToGlobalFrame(const PoseStamped& pose_msg, PoseStamped& transformed)
{
	std::string global_frame = "/map";
	tf::Stamped<tf::Pose> pose_tf, global_tf;
	poseStampedMsgToTF(pose_msg, pose_tf);

	try {
		tf_.transformPose(global_frame, pose_tf, global_tf);
	}
	catch(tf::TransformException& ex) {
		ROS_WARN_THROTTLE(2,"[person_tracker] Failed to transform goal pose from \"%s\" to \"%s\" frame: %s",
		pose_tf.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return false;
	}

	tf::poseStampedTFToMsg(global_tf, transformed);
	return true;
}

void
PersonTracker::skeletonCB(const body_msgs::Skeletons& skel_msg)
{
	ROS_INFO_THROTTLE(5,"[person tracker] Got data, %lu skeletons.", skel_msg.skeletons.size());

	body_msgs::SkeletonJoint body_part;
	string                   body_part_name;
	
	std::vector<double> thresholds;
	thresholds.push_back(0.7);
	//thresholds.push_back(0.5);
	//thresholds.push_back(0.3);
	//thresholds.push_back(-0.1);
	
	for( int ithresh = 0; ithresh<thresholds.size(); ithresh++ )
	{
		// Find the first trackable person
		for( unsigned int i=0; i<skel_msg.skeletons.size(); i++ )
		{
			// If the person has a trackable joint, save the location
			if( getFirstGoodJoint(skel_msg.skeletons.at(i), thresholds.at(ithresh), body_part, body_part_name) )
			{
				ROS_INFO_THROTTLE(2,"[person tracker] Player %d's %s has confidence %f", skel_msg.skeletons.at(i).playerid, body_part_name.c_str(), body_part.confidence);
			
				// Create a new pose
				PoseWithCovarianceStamped temp_pose;
				temp_pose.pose.pose.position    = body_part.position;
				temp_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				temp_pose.header.frame_id       = skel_msg.header.frame_id;
				temp_pose.header.stamp          = skel_msg.header.stamp;
				
				setVariance(temp_pose, 0.0);
				
				// Transform the goal to a global, fixed frame.  Otherwise it will be at most detect_timeout old which is really bad in an odometric frame
				PoseWithCovarianceStamped transformed_pose;
				if( poseToGlobalFrame(temp_pose, transformed_pose) )
				{
					person_pos_ = transformed_pose;
					last_detect_ = ros::Time::now();
					break;
				}
			}
		}
	}
}

void
PersonTracker::computeStateLoop(const ros::TimerEvent& event) {
	ROS_DEBUG("[person tracker] Last callback took %f seconds", event.profile.last_duration.toSec());

	// Set variance based on time since last acquisition
	double dt = timeSinceLastDetect().toSec();
	double current_var  = std::min(var_increase_rate_*dt, max_var_);
	setVariance(person_pos_, current_var);

	// Publish the goal and a stripped-down version without the covariance
	goal_cov_pub_.publish( person_pos_ );
	goal_pub_    .publish( stripCovariance(person_pos_) );
	
	ROS_INFO_THROTTLE(2, "[person_tracker] Time since detect = %.2fs, Variance = %.2fm^2", dt, current_var);
	
	// Update the sound player
	if( hasPerson() ) {
		sound_player_.setState(PTSounds::state_tracking);
	}
	else {
		sound_player_.setState(PTSounds::state_searching);
	}
	
	sound_player_.update();
	ros::spinOnce();
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
