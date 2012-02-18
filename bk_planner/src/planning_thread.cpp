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
			ROS_INFO("[planning] Starting recovery");
			escalatePlannerState(IN_RECOVERY);
			startRecovery();
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
			ROS_INFO("[planning] Planner state good, committing path segments.");
			commitPathSegments();
			enableFeeder();
		}
	
		boost::this_thread::sleep(boost::posix_time::milliseconds(period_ms));
	}
}

void BKPlanner::startRecovery()
{

}

void BKPlanner::doRecovery()
{

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
	
	precision_navigation_msgs::Path segment_plan;
	bool success = planPointToPoint(start, goal, segment_plan);
	
	// Get safe velocities for the segments
	path_checker_->assignPathVelocity(segment_plan);
	
	// Have the visualizer publish visualization
	segment_visualizer_->publishVisualization(segment_plan);
	
	// Temporary: execute the whole plan
	client_.waitForServer();
	precision_navigation_msgs::ExecutePathGoal action_goal;
	action_goal.segments = segment_plan.segs;
	client_.sendGoal(action_goal);
	
	return success;
}

bool BKPlanner::doPartialReplan()
{
	return false;
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

}

};//namespace
