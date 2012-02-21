#include <bk_planner/bk_planner.h>

// Thread-safe data access functions
namespace bk_planner {

void
BKPlanner::setFeederEnabled(bool state)
{
	boost::recursive_mutex::scoped_lock l(committed_path_mutex_);
	feeder_enabled_ = state;
}

bool
BKPlanner::isFeederEnabled()
{
	boost::recursive_mutex::scoped_lock l(committed_path_mutex_);
	return feeder_enabled_;
}

void 
BKPlanner::sendResetSignals()
{
//	ROS_INFO("Resetting");
	// Wait for the feeder to catch up and finish its main loop
	boost::recursive_mutex::scoped_try_lock l(feeder_lock_mutex);
	
	while( !l )
	{
		l = boost::recursive_mutex::scoped_try_lock(feeder_lock_mutex);
//		ROS_INFO("Resetting");
		ros::Duration(.1).sleep();
	}
	
	// Disable the feeder
	setFeederEnabled(false);
//	ROS_INFO("Reset done");
}

// Returns the a snapshot of linear distance left for the feeder
precision_navigation_msgs::ExecutePathFeedback last_fb_checked_;
double
BKPlanner::getFeederDistLeft()
{
	bool stale_fb = false;
	
	p_nav::Path p;
	int current_segnum;
	double current_seg_complete = 0.0;
	double d = 0.0;
	
	{
		boost::recursive_mutex::scoped_lock l1(feeder_path_mutex_);
		p = feeder_path_;
	}
	
	{
		boost::mutex::scoped_lock           l2(feedback_mutex_);
		
		// Check to see if we checked the same feedback twice in a row
		/*if( last_fb_checked_.seg_distance_done          == latest_feedback_.seg_distance_done
		 && last_fb_checked_.current_segment.seg_number == latest_feedback_.current_segment.seg_number){
		 
		 	// Additional check: at the beginning of a path, the feedback is never stale
		 	if(current_seg_complete = 0.0)
		 		stale_fb = false;
		 	else
				stale_fb = true;
		}*/
		last_fb_checked_ = latest_feedback_;
		
		current_segnum       = latest_feedback_.current_segment.seg_number;
		current_seg_complete = latest_feedback_.seg_distance_done;
	}
	
	
	if( stale_fb ) {
		return 1000.0;
	}
	
	if( p.segs.size() == 0 ){
		return 0.0;
	}
	
	int start_idx = segment_lib::segnumToIndex(p, current_segnum);
	
	// Length of first segment
	d = segment_lib::linDist(p.segs.front());
	d = max(0.0, d-current_seg_complete);
	
	
	if( p.segs.size() > 1)
	{
		// Length of the rest
		for( int i = start_idx + 1; i < p.segs.size(); i++ )
		{
			d += segment_lib::linDist(p.segs.at(i));
		}
	}
	
	return d;
}

bool
BKPlanner::segmentsAvailable()
{
	boost::recursive_mutex::scoped_lock l(committed_path_mutex_);
	int num_available = committed_path_.segs.size();
	return num_available > 0;
}

precision_navigation_msgs::Path
BKPlanner::dequeueSegments()
{
	boost::recursive_mutex::scoped_lock l(committed_path_mutex_);
	
	// Return the current committed path, and clear it.
	precision_navigation_msgs::Path segs_to_return = committed_path_;
	committed_path_.segs.clear();
	
	return segs_to_return;
}

void
BKPlanner::enqueueSegments(precision_navigation_msgs::Path new_segments)
{
	boost::recursive_mutex::scoped_lock l(committed_path_mutex_);
	
	// No path exists - commit the new one
	if( committed_path_.segs.size() == 0 ) {
		committed_path_ = new_segments;
		return;
	}
	
	// A path already exists.  Make sure the segment numbers are continuous
	if(committed_path_.segs.back().seg_number+1 != new_segments.segs.front().seg_number){
		ROS_ERROR("Tried to commit discontinuous segments");
		return;
	}
	
	// Append new segments to current ones
	committed_path_.segs.insert(committed_path_.segs.end(),
	                            new_segments.segs.begin(), new_segments.segs.end() );
}

void 
BKPlanner::setNewGoal(geometry_msgs::PoseStamped new_goal)
{
	boost::recursive_mutex::scoped_lock l(goal_mutex_);
	got_new_goal_ = true;
	latest_goal_  = new_goal;
}

bool 
BKPlanner::gotNewGoal()
{
	boost::recursive_mutex::scoped_lock l(goal_mutex_);
	return got_new_goal_;
}

geometry_msgs::PoseStamped 
BKPlanner::getLatestGoal()
{
	boost::recursive_mutex::scoped_lock l(goal_mutex_);
	got_new_goal_ = false;
	return latest_goal_;
}

void 
BKPlanner::escalatePlannerState(plannerState newstate)
{
	boost::recursive_mutex::scoped_lock l(planner_state_mutex_);
	if( newstate > planner_state_ )
		planner_state_ = newstate;
}

void 
BKPlanner::setPlannerState(plannerState newstate)
{
	boost::recursive_mutex::scoped_lock l(planner_state_mutex_);
	planner_state_ = newstate;
}

plannerState 
BKPlanner::getPlannerState()
{
	boost::recursive_mutex::scoped_lock l(planner_state_mutex_);
	return planner_state_;
}

};//namespace
