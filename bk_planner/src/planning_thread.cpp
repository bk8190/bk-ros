#include <bk_planner/bk_planner.h>

namespace bk_planner {

void
BKPlanner::runPlanningThread()
{
	long period_ms    = (double)1000 * 0.5; // 1/planner_frequency_;
	long wait_time_ms = (double)1000 * 0.5;
	bool made_one_plan = false; // true if we made at least one plan
	ros::Rate r(1.0); // hz
	
	ROS_INFO("[planning] bk_planner planning thread started, period is %ld, wait time %ld", period_ms, wait_time_ms);
	
		
	// For visualizing the planning in progress
	planner_visualizer_ = boost::shared_ptr<segment_lib::SegmentVisualizer>
		(new segment_lib::SegmentVisualizer(std::string("planning_visualization")) );
		
	// Main loop
	while(ros::ok())
	{
	
		if( getPlannerState() == IN_RECOVERY )
		{
			doRecovery();
		}
		else if( getPlannerState() == NEED_RECOVERY )
		{
			startRecovery();
			escalatePlannerState(IN_RECOVERY);
		}
		else if( getPlannerState() == NEED_FULL_REPLAN )
		{
			sendResetSignals(); // bring the system to a halt
			
			if( doFullReplan() ) {
				setPlannerState(GOOD);
				made_one_plan = true;
			}else {
				escalatePlannerState(NEED_RECOVERY);
			}
		}
		else if( getPlannerState() == NEED_PARTIAL_REPLAN )
		{
			
			if( doPartialReplan() ){
				setPlannerState(GOOD);
				made_one_plan = true;
			}else {
				escalatePlannerState(NEED_FULL_REPLAN);
			}
		}
		
		if( getPlannerState() == GOOD && made_one_plan )
		{
			bool path_clear = path_checker_->isPathClear(planner_path_);
			bool segs_left  = planner_path_.segs.size()>0;
			// Check if the path is clear.  If so, pass segments down to the feeder.
			if( path_clear && segs_left )
			{
				ROS_INFO_THROTTLE(5, "[planning] Planner state good.");
				commitPathSegments();
				setFeederEnabled(true);
			}
			else if(segs_left)
			{
				ROS_INFO_THROTTLE(2, "[planning] Found obstacles.");
				escalatePlannerState(NEED_PARTIAL_REPLAN);
			}
			else
			{
				ROS_INFO_THROTTLE(5, "[planning] Nothing more to pass down.");
			}
		}
	
		// We are planning in the odometry frame, which constantly is shifting.  Lie and say the plan was created right now to avoid using an old transform.
		if( planner_path_.segs.size() > 0 ){
			planner_path_.segs.back().header.stamp = ros::Time::now();
			segment_lib::reFrame(planner_path_);
		}
		else{
			planner_path_.header.stamp = ros::Time::now();
		}
		
		// publish visualization
		planner_visualizer_->publishVisualization(planner_path_);
		
		boost::this_thread::interruption_point();
		r.sleep();
	}
}

void
BKPlanner::startRecovery()
{
	ROS_INFO("[planning] Starting recovery");
}

void
BKPlanner::doRecovery()
{
	ROS_INFO("[planning] In recovery");
	
	// Temporary: just wait for a new goal
	if(got_new_goal_) {
		setPlannerState(NEED_FULL_REPLAN);
	}
}

bool
BKPlanner::doFullReplan()
{	
	ROS_INFO("[planning] Doing full replan");
			
	// Get the robot's current pose
	tf::Stamped<tf::Pose> robot_pose;
	if(!planner_costmap_->getRobotPose(robot_pose)){
		ROS_ERROR("[planning] bk_planner cannot make a plan for you because it could not get the start pose of the robot");
		return false;
	}
	PoseStamped start;
	tf::poseStampedTFToMsg(robot_pose, start);
	
	// Get the goal
	PoseStamped goal = getLatestGoal();
	
	// Clear current path
	dequeueSegments();
	planner_path_.segs.clear();
	
	// Plan to the goal
	//ROS_INFO("[planning] Full replan started...");
	bool success = planApproximateToGoal(start, goal, planner_path_);
	//ROS_INFO("[planning] Done.");
	
	// Reindex the path so that all previously committed segments are invalid
	segment_lib::reindexPath(planner_path_, last_committed_segnum_ + 2);
	
	planner_path_.header.stamp = planner_path_.segs.back().header.stamp;
	
	if( !success ){
		ROS_INFO("[planning] Full replan failed");
	}
	
	return success;
}

bool
BKPlanner::doPartialReplan()
{
	ROS_INFO("[planning] Doing partial replan");
			
	// If the feeder doesn't have any distance left to travel, do a full replan instead.
	double dist_left = getFeederDistLeft();
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
	
	PoseStamped goal  = getLatestGoal();
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
	bool success = planApproximateToGoal(start, goal, splice);
	
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


// Point-to-point planner
bool
BKPlanner::planPointToPoint(const PoseStamped& start,
                            const PoseStamped& goal,
                            p_nav::Path&       path)
{
  // Make sure the goal is clear
  if( !path_checker_->isPoseClear(start) )
  {
  	ROS_INFO("[planning] Start pose blocked!");
  	//bool success = getNearestClearGoal(start, goal);
  	return false;
  } 
  
  // Make sure the goal is in bounds
  if( !path_checker_->isPoseClear(goal) )
  {
  	ROS_INFO("[planning] Goal pose blocked!");
  	return false;
  } 
   
  ros::Time t1 = ros::Time::now();
	bool success = lattice_planner_->makeSegmentPlan(start, goal, path);
	ros::Duration t = ros::Time::now() - t1;
	
	ROS_INFO("[planning] Point-point plan made in %.2f seconds. Plan has %lu segments.", t.toSec(), path.segs.size());
	
	if( success == false ){
		ROS_ERROR("[planning] Point-point plan failed!");
		return false;
	}
	
	return true;
}

bool
BKPlanner::planApproximateToGoal(const PoseStamped& start,
                                 const PoseStamped& goal,
                                 p_nav::Path&       path)
{
	// Make a vector of poses (true goal included)
	// Put the most likely ones in front
	vector<PoseStamped> potential_goals = generatePotentialGoals(goal);
	
	
  if( cleared_goals.size() == 0 ) {
  	ROS_INFO("[planning] All potential goal poses blocked!");
  	return false;
  } 
	
	bool found_goal = false;
	p_nav::Path our_path;
	
	for( int igoal=0; igoal<cleared_goals.size(); igoal++ )
	{
		found_goal = planPointToPoint(start, cleared_goals.at(igoal), our_path);
		
		// Take the first found goal
		if( found_goal ){
			break;
		}
	}
	
	// Success
	if( found_goal ){
		path = our_path;
		return true;
	}
	
	// Failure
	path = p_nav::Path;
	ROS_INFO("[planning] Searched through %d candidate poses, did not find a path to goal", cleared_goals.size());
	return false;
}

void
BKPlanner::commitPathSegments()
{
	boost::recursive_mutex::scoped_try_lock l(committed_path_mutex_);
	while(!l){
		l = boost::recursive_mutex::scoped_try_lock(committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	p_nav::PathSegment seg_just_committed;
	double dist_just_committed;
	
	// How much path the feeder has left
	double dist_left = getFeederDistLeft();
	
	// Desired distance to add
	double dist_to_add = commit_distance_ - dist_left;
	
	//ROS_INFO("[planning] Feeder has %.2f left, committing %.2f, I have %d segs left", dist_left, dist_to_add, planner_path_.segs.size());
	while( planner_path_.segs.size() > 0 && dist_to_add > 0 )
	{
		seg_just_committed  = commitOneSegment();
		dist_just_committed = segment_lib::linDist(seg_just_committed);
		dist_to_add        -= dist_just_committed;
	}
	//ROS_INFO("Done committing");
}

p_nav::PathSegment
BKPlanner::commitOneSegment()
{
	p_nav::Path path_to_commit;
	path_to_commit.header = planner_path_.header;
	
	// Commit this segment and get rid of it
	p_nav::PathSegment seg = planner_path_.segs.front();
	planner_path_.segs.erase(planner_path_.segs.begin());
	
	// Remember our last committed pose
	last_committed_pose_   = segment_lib::getEndPose(seg);
	last_committed_segnum_ = seg.seg_number;
	
	path_to_commit.segs.push_back(seg);
	enqueueSegments(path_to_commit);
	//ROS_INFO("[planning] Planner committed segment %d", last_committed_segnum_);
	return seg;
}

PoseStamped getPoseOffset(const PoseStamped& pose, double dtheta, double dx)
{
	PoseStamped newpose;
	newpose.header     = pose.header;
	newpose.position.x = pose.position.x + cos(theta)*dx;
	newpose.position.y = pose.position.y + sin(theta)*dx;
	newpose.position.z = pose.position.z;
	newpose.orientation = tf::createQuaternionMsgFromYaw(  tf::getYaw(pose.orientation) + dtheta );

	return newpose;
}

// Generates candidate goals centered on the true goal
vector<PoseStamped> generatePotentialGoals(const PoseStamped& true_goal)
{
	vector<PoseStamped> potential_goals;
	PoseStamped         new_goal;
	double standoff_distance_ = 0.5;
	
	// First of all, add the true goal.
	goals.push_back(true_goal);
	
	double x0 = true_goal.position.x;
	double y0 = true_goal.position.y;
	double t0 = tf::getYaw(true_goal.orientation);
	double theta;
	
	// Add a goal in behind the true goal
	new_goal = getPoseOffset(true_goal, 0, -1.0 * standoff_distance_);
	potential_goals.push_back(new_goal);
	
	// Add a goal to the left of the true goal, facing inward
	new_goal = getPoseOffset(true_goal, segment_lib::pi/2.0, -1.0 * standoff_distance_);
	potential_goals.push_back(new_goal);
	
	// Add a goal to the right of the true goal, facing inward
	new_goal = getPoseOffset(true_goal, -1.0*segment_lib::pi/2.0, -1.0 * standoff_distance_);
	potential_goals.push_back(new_goal);
	
	// Eliminate those in collision
	vector<PoseStamped> cleared_goals = path_checker_->getGoodPoses(potential_goals);
	
	// Publish the candidate goals
	geometry_msgs::PoseArray pub_goals;
	pub_goals.header = true_goal.header;
	for(int i=0; i<cleared_goals.size(); i++){
		pub_goals.poses.push_back(cleared_goals.at(i).pose);
	}
	candidate_goal_pub_.publish(pub_goals);
	
	return cleared_goals;
}
			
};//namespace
