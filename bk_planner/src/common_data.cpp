#include <bk_planner/bk_planner.h>

namespace bk_planner {

void
BKPlanner::enableFeeder()
{
}

void
BKPlanner::disableFeeder()
{
}

void 
BKPlanner::sendResetSignals()
{

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
