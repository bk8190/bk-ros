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
	setFeederEnabled(false);
}


bool
BKPlanner::segmentsAvailable()
{
	boost::recursive_mutex::scoped_lock l(committed_path_mutex_);
	return (committed_path_.segs.size() > 0);
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
