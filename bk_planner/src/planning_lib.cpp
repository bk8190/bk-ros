#include <bk_planner/bk_planner.h>
namespace bk_planner {

bool BKPlanningThread::isTargetBehind(const PoseStamped& target)
{
	// Transform the target to base_link
	PoseStamped transformed_target;
	try {
		parent_->tf_.transformPose("base_link", target, transformed_target);
	}
	catch(tf::TransformException& ex) {
		ROS_WARN("[isTargetBehind] Failed to transform target pose from \"%s\" to \"base_link\" (%s)",
		target.header.frame_id.c_str(), ex.what());
		return false;
	}
	
	//ROS_INFO("[isTargetBehind] Transformed pose is (%.2f,%.2f)", transformed_target.pose.position.x, transformed_target.pose.position.y);
	
	return transformed_target.pose.position.x < -2.0;
}


bool
BKPlanningThread::doFullReplan()
{	
	ROS_INFO("[planning] Doing full replan");
	
	// Get the robot's current pose
	PoseStamped start;
	if(!parent_->getRobotPose(start)){
		ROS_ERROR("[planning] Full replan failed (could not get robot's current pose)");
		return false;
	}

	// Clear obstacles from the robot's footprint
	parent_->planner_costmap_->clearRobotFootprint();
	
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
		segment_lib::combineSegments(planner_path_);
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
	
	// Clear obstacles from the robot's footprint
	parent_->planner_costmap_->clearRobotFootprint();
	
	// If the feeder doesn't have any distance left to travel, do a full replan instead.
	double dist_left = FeedThreadPtr(feeder_)->getFeederDistLeft();
	// ROS_INFO("[planning] Feeder has %.2fm left", dist_left);
	if( dist_left < 0.1 )//&& planner_path_.segs.size() == 0)
	{
		ROS_INFO("[planning] Feeder path empty and nothing more to commit, doing full replan instead.");
		return false;
	}
	
	PoseStamped goal  = parent_->getLatestGoal();
	
	// Trigger full replan if target has moved behind the robot.  There is a timer to prevent this from happening too often
	if( isTargetBehind(goal) && (ros::Time::now() - last_scrapped_path_ > scrap_path_timeout_) )
	{
		ROS_INFO("[planning] Target is behind me, scrapping current path and doing a full replan.");
		last_scrapped_path_ = ros::Time::now();
		return false;
	}
	
	// Clear all but the first uncommitted segment
	if( planner_path_.segs.size() > 1 ) {
		planner_path_.segs.erase( planner_path_.segs.begin()+1, planner_path_.segs.end() );
	}
	
	bool replan_from_end_of_first_seg;
		
	// Plan from the end of the first uncommitted segment if possible, otherwise plan from the last committed pose
	PoseStamped start;
	
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
		segment_lib::combineSegments(planner_path_);
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
  // Make sure the start pose is clear
  /*if( !path_checker->isPoseClear(start) )
  {
  	ROS_INFO("[planning] Start pose blocked!");
  	return false;
  }*/
  
  // Make sure the goal pose is clear
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
BKPlanningThread::planShortDistance(const PoseStamped& start,
                                    const PoseStamped& goal,
                                    p_nav::Path&       path,
                                    shared_ptr<path_checker::PathChecker> path_checker)
{
	// Get the angle to the target
	double dx    = goal.pose.position.x - start.pose.position.x;
	double dy    = goal.pose.position.y - start.pose.position.y;
	double theta = atan2(dy,dx);
	
	// Instruct the robot to rotate in place
	PoseStamped newgoal = start;
	newgoal.header = start.header;
	newgoal.pose.position = start.pose.position;
	newgoal.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  // Make sure the goal is in bounds
  if( !path_checker->isPoseClear(newgoal) )
  {
  	ROS_INFO("[planning] [Short distance planner] Goal pose blocked!");
  	return false;
  }
  
	// Construct a turn-in-place segment
	double x0 = start.pose.position.x;
	double y0 = start.pose.position.y;
	double t1 = tf::getYaw(start  .pose.orientation);
	double t2 = tf::getYaw(newgoal.pose.orientation);
	
	p_nav::PathSegment p = segment_lib::makeTurnSegment(x0, y0, t1, t2);
	
	path.segs.clear();
	path.segs.push_back(p);
	
	goal_pub_.publish(newgoal);
	return true;
}

bool
BKPlanningThread::planApproximateToGoal(const PoseStamped& start,
                                        const PoseStamped& goal,
                                        p_nav::Path&       path,
                                        shared_ptr<path_checker::PathChecker>     path_checker,
                                        shared_ptr<bk_sbpl::BKSBPLLatticePlanner> lattice_planner)
{
	bool found_goal = false;
	
	double d = dist(start,goal);
	
	// See if the target is close.  If so, activate a special short-distance planner.
	if( d <= short_dist_ && getPlannerState() == NEED_FULL_REPLAN && !override_goal_generation_)
	{
		found_goal = planShortDistance(start, goal, path, path_checker);
	
		// Success in short-distance planner (fallthrough to normal planner on failure)
		if( found_goal ){
			ROS_INFO("[planning] Short-distance planner succeeded");
			return true;
		}
	}

	// Generate potential goal poses
	vector<PoseStamped> cleared_goals = generatePotentialGoals(start, goal, path_checker);
	
  if( cleared_goals.size() == 0 ) {
  	ROS_INFO("[planning] All potential goal poses blocked!");
  	return false;
  } 
	
	for( unsigned int igoal=0; igoal<cleared_goals.size(); igoal++ )
	{
		found_goal = planPointToPoint(start, cleared_goals.at(igoal), path, path_checker, lattice_planner);
		
		// Take the first valid path
		if( found_goal ){
			goal_pub_.publish(cleared_goals.at(igoal));
			return true;
		}
	}
	
	// Failure
	path = p_nav::Path();
	ROS_INFO("[planning] Searched through %lu candidate goals, no clear path.", cleared_goals.size());
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

double dist( const PoseStamped& p1, const PoseStamped& p2 )
{
	return sqrt(
              ((p1.pose.position.x - p2.pose.position.x)
              *(p1.pose.position.x - p2.pose.position.x))+
              ((p1.pose.position.y - p2.pose.position.y)
              *(p1.pose.position.y - p2.pose.position.y))+
              ((p1.pose.position.z - p2.pose.position.z)
              *(p1.pose.position.z - p2.pose.position.z)) );
}

// Up to a certain threhsold, just turn and face the target.
vector<PoseStamped>
BKPlanningThread::generateNearGoals(const PoseStamped& start,
                                    const PoseStamped& goal,
                                    shared_ptr<path_checker::PathChecker> path_checker)
{
	// Get the angle to the target
	double dx    = goal.pose.position.x - start.pose.position.x;
	double dy    = goal.pose.position.y - start.pose.position.y;
	double theta = atan2(dy,dx);
	
	// Instruct the robot to rotate in place
	PoseStamped newgoal = start;
	newgoal.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
	
	vector<PoseStamped> potential_goals;
	potential_goals.push_back(newgoal);
	
	// Eliminate goals in collision
	vector<PoseStamped> cleared_goals = path_checker->getGoodPoses(potential_goals);
	
	
	// Store the candidate goals for visualization
	pub_goals_.poses.clear();
	pub_goals_.header = newgoal.header;
	for(unsigned int i=0; i<cleared_goals.size(); i++){
		pub_goals_.poses.push_back(cleared_goals.at(i).pose);
	}
	
	return cleared_goals;
}

PoseStamped last_robot_pose_;

bool poseCompare (PoseStamped i,PoseStamped j)
{
	return dist(i,last_robot_pose_) < dist(j,last_robot_pose_);
}

vector<PoseStamped>
BKPlanningThread::generateFarGoals(const PoseStamped& start,
                                   const PoseStamped& goal,
                                   shared_ptr<path_checker::PathChecker> path_checker)
{
	// Modify the goal's angle
	double dx = goal.pose.position.x - start.pose.position.x;
	double dy = goal.pose.position.y - start.pose.position.y;
	PoseStamped newgoal = goal;
	newgoal.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy,dx));
	
	vector<PoseStamped> potential_goals;
	double ds = -1.0*standoff_distance_;
	
	// First of all, add the true goal.
	//potential_goals.push_back(goal);
	
	// Add a goal in between the robot and the true goal
	potential_goals.push_back(getPoseOffset(newgoal,  0.00  , ds));
	
	potential_goals.push_back(getPoseOffset(newgoal,  .1*pi, ds*0.9));
	potential_goals.push_back(getPoseOffset(newgoal, -.1*pi, ds*0.9));
	
	// Add two goals behind the true goal, offset by +-45 degrees
	potential_goals.push_back(getPoseOffset(newgoal,  .2*pi, ds*.8));
	potential_goals.push_back(getPoseOffset(newgoal, -.2*pi, ds*.8));
	
	potential_goals.push_back(getPoseOffset(newgoal,  .05*pi, ds*1.3));
	potential_goals.push_back(getPoseOffset(newgoal, -.05*pi, ds*1.3));
	
	// Add two goals behind the true goal, offset by +-70 degrees
	potential_goals.push_back(getPoseOffset(newgoal,  .4*pi, ds*.7));
	potential_goals.push_back(getPoseOffset(newgoal, -.4*pi, ds*.7));
	
	// Add two goals behind the true goal, offset by +-135 degrees
	potential_goals.push_back(getPoseOffset(newgoal,  .8*pi, ds*1.0));
	potential_goals.push_back(getPoseOffset(newgoal, -.8*pi, ds*1.0));
	
	
	// Add a goal 180 degrees offset
	//potential_goals.push_back(getPoseOffset(newgoal, 1.00*pi, ds));
	
	// Eliminate goals in collision
	vector<PoseStamped> cleared_goals = path_checker->getGoodPoses(potential_goals);
	
	// Sort goals by distance to the robot
	//parent_->getRobotPose(last_robot_pose_);
	//sort (cleared_goals.begin(), cleared_goals.end(), poseCompare);
	
	// Keep only the first 4 goals to keep the planning time reasonable
	if( getPlannerState() == NEED_PARTIAL_REPLAN ) {
		while( cleared_goals.size() > 4 )	{
			cleared_goals.erase(cleared_goals.end());
		}
	}
	/*else if( getPlannerState() == NEED_FULL_REPLAN ) {
		while( cleared_goals.size() > 7 )	{
			cleared_goals.erase(cleared_goals.end());
		}
	}*/
	
	// Store the candidate goals for visualization
	pub_goals_.poses.clear();
	pub_goals_.header = newgoal.header;
	for(unsigned int i=0; i<cleared_goals.size(); i++){
		pub_goals_.poses.push_back(cleared_goals.at(i).pose);
	}
	
	return cleared_goals;
}

// Generates candidate goals centered on the true goal
vector<PoseStamped>
BKPlanningThread::generatePotentialGoals(const PoseStamped& start,
                                         const PoseStamped& goal,
                                         shared_ptr<path_checker::PathChecker> path_checker)
{
	double d = dist(start,goal);
	vector<PoseStamped> cleared_goals;

	// Optionally, just return the true goal (useful for directly testing point-point planning)
	if( override_goal_generation_ ) {
		cleared_goals.push_back(goal);
		return cleared_goals;
	}

	if( d < short_dist_ )
	{
		// Try the short-distance planner.  If it fails, use the long-distance planner.
		cleared_goals = generateNearGoals(start, goal, path_checker);
		
		if( cleared_goals.size() > 0 ){
			//ROS_INFO("[planning] Near goal");
			return cleared_goals;
		}
	}
	
	// Either we are out of range of the short-term distance or the short distance failed
	cleared_goals = generateFarGoals(start, goal, path_checker);
	//ROS_INFO("[planning] Far goal");
	return cleared_goals;
}
};//namespace
