#include <bk_planner/bk_planner.h>

namespace bk_planner {

void BKPlanner::runPlanningThread()
{
	long period_ms    = (double)1000 * 0.5; // 1/planner_frequency_;
	long wait_time_ms = (double)1000 * 0.5;
	
	ROS_INFO("[planning] bk_planner planning thread started, period is %ld, wait time %ld", period_ms, wait_time_ms);
	
	while(true)
	{
		if( getPlannerState() == IN_RECOVERY )
		{
			ROS_INFO("[planning] In recovery");
			doRecovery();
		}
		else if( getPlannerState() == NEED_RECOVERY )
		{
			/*ROS_INFO("[planning] Starting recovery");
			escalatePlannerState(IN_RECOVERY);
			startRecovery();*/
			
			// HACK: Recovery state does not exist yet
			ROS_INFO("[planning] Recovery state does not exist, setting state to \"full replan\"");
			setPlannerState(NEED_FULL_REPLAN);
		}
		else if( getPlannerState() == NEED_FULL_REPLAN )
		{
			ROS_INFO("[planning] Doing full replan");
			sendResetSignals();
			
			if( doFullReplan() ) {
				setPlannerState(GOOD);
			}else {
				ROS_INFO("[planning] Full replan failed, signaling recovery");
				escalatePlannerState(NEED_RECOVERY);
			}
		}
		else if( getPlannerState() == NEED_PARTIAL_REPLAN )
		{
			ROS_INFO("[planning] Doing partial replan");
			
			if( doPartialReplan() ){
				setPlannerState(GOOD);
			}else {
				ROS_INFO("[planning] Partial replan failed, signaling full replan");
				escalatePlannerState(NEED_FULL_REPLAN);
			}
		}
		
		if( getPlannerState() == GOOD )
		{
			//ROS_INFO("[planning] Planner state good.");
			commitPathSegments();
			setFeederEnabled(true);
		}
	
		boost::this_thread::sleep(boost::posix_time::milliseconds(period_ms));
	}
}

void BKPlanner::startRecovery()
{
	return;
}

void BKPlanner::doRecovery()
{
	return;
}

bool BKPlanner::doFullReplan()
{	
	// Get the robot's current pose
	tf::Stamped<tf::Pose> robot_pose;
	if(!planner_costmap_->getRobotPose(robot_pose)){
		ROS_ERROR("[planning] bk_planner cannot make a plan for you because it could not get the start pose of the robot");
		return false;
	}
	geometry_msgs::PoseStamped start;
	tf::poseStampedTFToMsg(robot_pose, start);
	
	// Get the goal
	geometry_msgs::PoseStamped goal = getLatestGoal();
	
	// Plan to the goal
	bool success = planPointToPoint(start, goal, planner_path_);
	
	// Reindex the path so that all previous segments are invalid
	segment_lib::reindexPath(planner_path_, last_valid_segnum_ + 2);
	
	// Update our internal path indices
	first_valid_segnum_    = segment_lib::getFirstSegnum(planner_path_);
	last_valid_segnum_     = segment_lib::getLastSegnum(planner_path_);
	last_committed_segnum_ = first_valid_segnum_-1;
	
	return success;
}

bool BKPlanner::doPartialReplan()
{
	// Get the last committed pose
	int start_idx = segment_lib::segnumToIndex(planner_path_, last_committed_segnum_+1);
	
	if( start_idx < 0 ){
		return false;
	}
	
	geometry_msgs::PoseStamped start = segment_lib::getEndPose(planner_path_.segs.at(start_idx));
	
	// Get the goal
	geometry_msgs::PoseStamped goal = getLatestGoal();
	
	// Plan to the goal
	bool success = planPointToPoint(start, goal, planner_path_);
	
	// Reindex the path, keeping previous segments valid
	segment_lib::reindexPath(planner_path_, first_valid_segnum_);
	
	// Update our internal path indices
	last_valid_segnum_     = segment_lib::getLastSegnum(planner_path_);
	
	return success;
}

// Point/point planner
bool BKPlanner::planPointToPoint(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 precision_navigation_msgs::Path&  segment_plan)
{
	/*if( !start.header.frame_id.compare(goal.header.frame_id) ){
		ROS_ERROR("Start and goal poses are in different frames.  What are you trying to pull here?");
		return false;
	}*/
    
  ros::Time t1 = ros::Time::now();
	bool success = lattice_planner_->makeSegmentPlan(start, goal, segment_plan);

	if( success == false ){
		ROS_ERROR("[planning] Planner failed!");
		return false;
	}
	ros::Duration t = ros::Time::now() - t1;
	
	ROS_INFO("[planning] Point-point plan made in %.2f seconds. Plan has %lu segments.", t.toSec(), segment_plan.segs.size());
	
	segment_plan.header.stamp    = ros::Time::now();
	segment_plan.header.frame_id = start.header.frame_id;
	
	return true;
}

void BKPlanner::commitPathSegments()
{
	// Check if we can commit some more
	for( int i=0; i<2; i++ )
	{
		if( canCommitOneSegment() ) {
			commitOneSegment();
		}
	}
}

// Check if we can commit another segment
bool BKPlanner::canCommitOneSegment()
{
	return(last_committed_segnum_ < last_valid_segnum_);
}

void BKPlanner::commitOneSegment()
{
	precision_navigation_msgs::Path        newsegs;
	precision_navigation_msgs::PathSegment seg;
	newsegs.header = planner_path_.header;
	
	// Try to find the index of the next segment scheduled to be committed
	int seg_idx = segment_lib::segnumToIndex(planner_path_, last_committed_segnum_+1);
	
	// if it exists, commit it
	if( seg_idx != -1 )
	{
		last_committed_segnum_++;	
		newsegs.segs.push_back( planner_path_.segs.at(seg_idx));
	}
	else
	{
		ROS_ERROR("[planning] Tried to commit a path segment (segnum %d) that didn't exist", last_committed_segnum_+1);
	}
	
	if( newsegs.segs.size() > 0 )
	{
		//ROS_INFO("[planning] Planner committed a path segment");
		enqueueSegments(newsegs);
	}
}

};//namespace
