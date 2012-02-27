#include <person_tracker/PersonTracker.h>

namespace PersonTracker {

PersonTracker::PersonTracker(string name) :
	nh_           (name),
	sound_player_ (nh_, "sounds"),
	found_person_ (false)
{
	nh_.param("loop_rate", loop_rate_, 2.0); // default 2Hz
	
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
	return(found_person_);
}

PoseStamped
PersonTracker::getPersonPosition()
{
	return(person_pos_);
}
		
void
PersonTracker::skeletonCB(const body_msgs::Skeletons& skel_msg)
{
	ROS_INFO("[person tracker] Got data, %lu skeletons.", skel_msg.skeletons.size());

	body_msgs::SkeletonJoint torso;
	found_person_ = false;
	
	// Find the first torso position
	for( unsigned int i=0; i<skel_msg.skeletons.size(); i++ )
	{
		// Get the torso position and the player's ID
		torso = skel_msg.skeletons.at(i).torso;
		
		ROS_INFO("[person tracker] Player %d has confidence %f", skel_msg.skeletons.at(i).playerid, torso.confidence);
		
		// If the confidence is high enough (>50%) save the location
		if( torso.confidence > 0.5 )
		{
			person_pos_.pose.position    = torso.position;
			person_pos_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			person_pos_.header.frame_id  = "camera_depth_optical_frame";
			person_pos_.header.stamp     = skel_msg.header.stamp;
			
			found_person_ = true;
			sound_player_.setState(PTSounds::state_tracking);
			
			break;
		}
	}
}

void
PersonTracker::computeStateLoop(const ros::TimerEvent& event) {
	ROS_DEBUG("[person tracker] Last callback took %f seconds", event.profile.last_duration.toSec());

	if( found_person_ ){
		goal_pub_.publish(getPersonPosition());
		ROS_INFO("[person tracker] Found a person at x=%.2f\ty=%.2f\tz=%.2f\t", person_pos_.pose.position.x, person_pos_.pose.position.y, person_pos_.pose.position.z);
	}
	else{
		sound_player_.setState(PTSounds::state_searching);
	}
	
	sound_player_.update();
}

void
clamp( double& x, const double low, const double high )
{
	if( x > high ) {
		x = high;
	}
	else if( x < low ) {
		x = low;
	}
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
