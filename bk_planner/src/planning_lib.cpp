#include <bk_planner/bk_planner.h>
namespace bk_planner {

bool
BKPlanningThread::doFullReplan()
{	
	ROS_INFO("[planning] Doing full replan");
			
	// Get the robot's current pose
	tf::Stamped<tf::Pose> robot_pose;
	if(!parent_->planner_costmap_->getRobotPose(robot_pose)){
		ROS_ERROR("[planning] Full replan failed (could not get robot's current pose)");
		return false;
	}
	PoseStamped start;
	tf::poseStampedTFToMsg(robot_pose, start);
	
	// Get the goal
	PoseStamped goal = parent_->getLatestGoal();
	
	// Clear any remnants of the current path
	dequeueSegments();
	planner_path_.segs.clear();
	
	// Plan to the goal
	bool success = planApproximateToGoal(start, goal, planner_path_, parent_->path_checker_, lattice_planner_);
	
	if( success )
	{
		// Reindex the path so that all previously committed segments are invalid
		segment_lib::reindexPath(planner_path_, last_committed_segnum_ + 2);
	
		planner_path_.header.stamp = planner_path_.segs.back().header.stamp;
		return true;
	}
	else
	{
		ROS_INFO("[planning] Full replan failed");
		return false;
	}
}

bool
BKPlanningThread::doPartialReplan()
{
	
	ROS_INFO("[planning] Doing partial replan");
			
	// If the feeder doesn't have any distance left to travel, do a full replan instead.
	double dist_left = FeedThreadPtr(feeder_)->getFeederDistLeft();
	// ROS_INFO("[planning] Feeder has %.2fm left", dist_left);
	if( dist_left < 0.1 && planner_path_.segs.size() == 0){
		ROS_INFO("[planning] Feeder path empty and nothing more to commit, doing full replan instead.");
		planner_path_.segs.clear();
		return false;
	}
		
	// Clear all but the first uncommitted segment
	if( planner_path_.segs.size() > 1 ) {
		planner_path_.segs.erase( planner_path_.segs.begin()+1, planner_path_.segs.end() );
	}
	
	PoseStamped goal  = parent_->getLatestGoal();
	PoseStamped start;
	
	bool replan_from_end_of_first_seg;
		
	// Plan from the end of the first uncommitted segment if possible, otherwise plan from the last committed pose
	if( planner_path_.segs.size() == 1 ){
		//ROS_INFO("[planning] Replanning from first uncommitted segment");
		start = segment_lib::getEndPose(planner_path_.segs.front());
		replan_from_end_of_first_seg = true;
	}
	else
	{
		//ROS_INFO("[planning] Replanning from last committed pose");
		replan_from_end_of_first_seg = false;
		start = last_committed_pose_;
	}
	
	// Make the plan and append it
	p_nav::Path splice;
	bool success = planApproximateToGoal(start, goal, splice, parent_->path_checker_, lattice_planner_);
	
	if( success ){
		if( replan_from_end_of_first_seg ){
			planner_path_.segs.insert(planner_path_.segs.begin()+1, splice.segs.begin(), splice.segs.end() );
		}else {
			planner_path_ = splice;
		}
		
		// Reindex the path to be continuous with the segments previously committed
		segment_lib::reindexPath(planner_path_, last_committed_segnum_+1 );
	
		planner_path_.header.stamp = planner_path_.segs.back().header.stamp;
		
		return true;
	}
	else{
		ROS_INFO("[planning] Partial replan failed, signaling full replan");
		return false;
	}
}

// Dumb point-to-point planner
bool
BKPlanningThread::planPointToPoint(const PoseStamped& start,
                                   const PoseStamped& goal,
                                   p_nav::Path&       path,
                                   shared_ptr<path_checker::PathChecker>     path_checker,
                                   shared_ptr<bk_sbpl::BKSBPLLatticePlanner> lattice_planner )
{
  // Make sure the goal is clear
  if( !path_checker->isPoseClear(start) )
  {
  	ROS_INFO("[planning] Start pose blocked!");
  	return false;
  } 
  
  // Make sure the goal is in bounds
  if( !path_checker->isPoseClear(goal) )
  {
  	ROS_INFO("[planning] Goal pose blocked!");
  	return false;
  } 
   
  ros::Time t1 = ros::Time::now();
	bool success = lattice_planner->makeSegmentPlan(start, goal, path);
	ros::Duration t = ros::Time::now() - t1;
	
	if( success == false ){
		ROS_WARN("[planning] Point-point plan failed! (took %.2f seconds)",t.toSec());
		return false;
	}
	
	ROS_INFO("[planning] Point-point plan made in %.2f seconds. Plan has %lu segments.", t.toSec(), path.segs.size());
	
	return true;
}
 
bool
BKPlanningThread::planApproximateToGoal(const PoseStamped& start,
                                        const PoseStamped& goal,
                                        p_nav::Path&       path,
                                        shared_ptr<path_checker::PathChecker>     path_checker,
                                        shared_ptr<bk_sbpl::BKSBPLLatticePlanner> lattice_planner)
{
	// Modify the goal's angle
	double dx = goal.pose.position.x - start.pose.position.x;
	double dy = goal.pose.position.y - start.pose.position.y;
	PoseStamped newgoal = goal;
	newgoal.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy,dx));
	
	// Generate potential goal poses
	vector<PoseStamped> cleared_goals = generatePotentialGoals(newgoal, path_checker);
	
  if( cleared_goals.size() == 0 ) {
  	ROS_INFO("[planning] All potential goal poses blocked!");
  	return false;
  } 
	
	bool found_goal = false;
	for( int igoal=0; igoal<cleared_goals.size(); igoal++ )
	{
		found_goal = planPointToPoint(start, cleared_goals.at(igoal), path, path_checker, lattice_planner);
		
		// Take the first valid path
		if( found_goal ){
			return true;
		}
	}
	
	// Failure
	path = p_nav::Path();
	ROS_INFO("[planning] Searched through %d candidate goals, no clear path.", cleared_goals.size());
	return false;
}

// Looks through path.  If a segment crosses a special boundary, adds a special segment, and replans from the special segment's start and endpoint.
/*p_nav::Path
BKPlanner::replaceSpecialSegments(const p_nav::Path& path)
{
	if(path.segs.size() == 0)
		return path;

	p_nav::Path newpath;
	newpath.header = path.header;
	
	PoseStamped last_endpoint = getStartPose(path.segs.front());
	PoseStamped goal          = getEndPose(path.segs.back());
	
	// Start at the beginning of the path
	// From last_endpoint, copy segments to temp until you find a special segment
	// If you encounter a special segment, replan from last_endpoint to start of special segs
	// insert special segs
	// set last_endpoint to end of special_segs
	// replan from last_endpoint to goal
	// From last_endpoint,
}*/

// Returns a pose offset from an original.
// Rotates in place by dtheta degrees and then moves dx meters foward.
PoseStamped
BKPlanningThread::getPoseOffset(const PoseStamped& pose, double dtheta, double dx)
{
	PoseStamped newpose;
	newpose.header          = pose.header;
	double new_angle = tf::getYaw(pose.pose.orientation) + dtheta;
	newpose.pose.position.x = pose.pose.position.x + cos(new_angle)*dx;
	newpose.pose.position.y = pose.pose.position.y + sin(new_angle)*dx;
	newpose.pose.position.z = pose.pose.position.z;
	newpose.pose.orientation = tf::createQuaternionMsgFromYaw(new_angle);

	return newpose;
}

// Generates candidate goals centered on the true goal
vector<PoseStamped>
BKPlanningThread::generatePotentialGoals(const PoseStamped& true_goal,
                                         shared_ptr<path_checker::PathChecker> path_checker)
{
	vector<PoseStamped> potential_goals;
	double ds = -1.0*standoff_distance_;
	
	// First of all, add the true goal.
	//potential_goals.push_back(true_goal);
	
	// Add a goal in between the robot and the true goal
	potential_goals.push_back(getPoseOffset(true_goal,  0.00  , ds));
	
	// Add two goals behind the true goal, offset by +-45 degrees
	potential_goals.push_back(getPoseOffset(true_goal,  .25*pi, ds));
	potential_goals.push_back(getPoseOffset(true_goal, -.25*pi, ds));
	
	// Add two goals behind the true goal, offset by +-90 degrees
	potential_goals.push_back(getPoseOffset(true_goal,  .50*pi, ds));
	potential_goals.push_back(getPoseOffset(true_goal, -.50*pi, ds));
	
	// Add two goals behind the true goal, offset by +-135 degrees
	//potential_goals.push_back(getPoseOffset(true_goal,  .75*pi, ds));
	//potential_goals.push_back(getPoseOffset(true_goal, -.75*pi, ds));
	
	// Add a goal 180 degrees offset
	//potential_goals.push_back(getPoseOffset(true_goal, 1.00*pi, ds));
	
	// Eliminate goals in collision
	vector<PoseStamped> cleared_goals = path_checker->getGoodPoses(potential_goals);
	
	// Store the candidate goals for visualization
	pub_goals_.poses.clear();
	pub_goals_.header = true_goal.header;
	for(int i=0; i<cleared_goals.size(); i++){
		pub_goals_.poses.push_back(cleared_goals.at(i).pose);
	}
	
	return cleared_goals;
}
};//namespace
