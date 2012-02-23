#include <bk_planner/bk_planner.h>

namespace bk_planner {

BKPlanningThread::BKPlanningThread(boost::weak_ptr<BKPlanner> parent):
	parent_weak_  (parent)
{
	shared_ptr<BKPlanner> parent_(parent_weak_);
	candidate_goal_pub_ = parent_->nh_.advertise<geometry_msgs::PoseArray>("candidate_poses", 1);

	// For visualizing the planning in progress
	visualizer_ = shared_ptr<segment_lib::SegmentVisualizer>
		(new segment_lib::SegmentVisualizer(std::string("planning_visualization")) );
		
	lattice_planner_ = shared_ptr<bk_sbpl::BKSBPLLatticePlanner>
		(new bk_sbpl::BKSBPLLatticePlanner("lattice_planner", parent_->planner_costmap_) );
	
	parent_->priv_nh_.param("planning/commit_distance" ,commit_distance_     ,1.1);
	parent_->priv_nh_.param("planning/standoff_distance" ,standoff_distance_ ,1.1);
	ROS_INFO("Commit   distance %.2f", commit_distance_);
	ROS_INFO("Standoff distance %.2f", standoff_distance_);
	
	planner_path_.segs.clear();
	planner_state_         = GOOD;
	last_committed_segnum_ = 0;
	last_committed_pose_   = parent_->getLatestGoal();
	
	ROS_INFO("[planning] Constructor finished");
}
	

void
BKPlanningThread::setFeeder(boost::weak_ptr<BKFeederThread> feeder)
{
	feeder_weak_ = feeder;
}

void
BKPlanningThread::run()
{
	ROS_INFO("[planning] Main thread started");
	bool made_one_plan = false; // true if we made at least one plan
	ros::Rate r(1.0); // hz
		
	while(ros::ok()) // Main loop
	{
		shared_ptr<BKFeederThread> feeder_(feeder_weak_);
		boost::this_thread::interruption_point();
	
		plannerState s = getPlannerState();
		switch(s)
		{
			case IN_RECOVERY:
				doRecovery();
				break;
				
			case NEED_RECOVERY:
				startRecovery();
				escalatePlannerState(IN_RECOVERY);
				break;
			
			case NEED_FULL_REPLAN:
				feeder_->sendResetSignals(); // bring the system to a halt
			
				if( doFullReplan() ) {
					setPlannerState(GOOD);
					made_one_plan = true;
				}else {
					escalatePlannerState(NEED_RECOVERY);
				}
				break;
				
			case NEED_PARTIAL_REPLAN:
				if( doPartialReplan() ){
					setPlannerState(GOOD);
					made_one_plan = true;
				}else {
					escalatePlannerState(NEED_FULL_REPLAN);
				}
				break;
				
			case GOOD:
				if( made_one_plan )
				{
					shared_ptr<BKPlanner> parent_(parent_weak_);
					bool path_clear = parent_->path_checker_->isPathClear(planner_path_);
					bool segs_left  = planner_path_.segs.size()>0;
			
					// Commit path segments if the path is clear and we have a good plan
					if( path_clear && segs_left )
					{
						ROS_INFO_THROTTLE(5, "[planning] Planner state good.");
						commitPathSegments();
						feeder_->setFeederEnabled(true);
					}
					else if(segs_left)
					{
						// We found obstacles, signla a replan
						ROS_INFO_THROTTLE(2, "[planning] Found obstacles.");
						escalatePlannerState(NEED_PARTIAL_REPLAN);
					}
					else
					{
						ROS_INFO_THROTTLE(5, "[planning] Nothing more to pass down.");
					}
				}
				break;	
			
			default:
				ROS_ERROR("[planning] Planning is in a bad state right now... State = %d",s);
				break;
		}
	
		// We are planning in the odometry frame, which constantly is shifting.  Lie and say the plan was created right now to avoid using an old transform.
		if( planner_path_.segs.size() > 0 ){
			planner_path_.segs.back().header.stamp = ros::Time::now();
			segment_lib::reFrame(planner_path_);
			
			// Also reframe the candidate goals for visualization
			pub_goals_.header.stamp = planner_path_.header.stamp;
			pub_goals_.header.frame_id = planner_path_.header.frame_id;
		}
		else{
			planner_path_.header.stamp = ros::Time::now();
		}
		
		// publish visualization
		visualizer_->publishVisualization(planner_path_);
		candidate_goal_pub_.publish(pub_goals_);
		
		boost::this_thread::interruption_point();
		r.sleep();
	}
}

void
BKPlanningThread::startRecovery()
{
	ROS_INFO("[planning] Starting recovery");
}

void
BKPlanningThread::doRecovery()
{
	shared_ptr<BKPlanner> parent_(parent_weak_);
	ROS_INFO("[planning] In recovery");
	
	// Temporary: just wait for a new goal
	if(parent_->gotNewGoal()) {
		setPlannerState(NEED_FULL_REPLAN);
	}
}

bool
BKPlanningThread::doFullReplan()
{	
	shared_ptr<BKPlanner> parent_(parent_weak_);
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
	bool success = planApproximateToGoal(start, goal, planner_path_);
	
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
	shared_ptr<BKPlanner> parent_(parent_weak_);
	shared_ptr<BKFeederThread> feeder_(feeder_weak_);
	
	ROS_INFO("[planning] Doing partial replan");
			
	// If the feeder doesn't have any distance left to travel, do a full replan instead.
	double dist_left = feeder_->getFeederDistLeft();
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


// Dumb point-to-point planner
bool
BKPlanningThread::planPointToPoint(const PoseStamped& start,
                            const PoseStamped& goal,
                            p_nav::Path&       path)
{
	shared_ptr<BKPlanner> parent_(parent_weak_);
  // Make sure the goal is clear
  if( !parent_->path_checker_->isPoseClear(start) )
  {
  	ROS_INFO("[planning] Start pose blocked!");
  	//bool success = getNearestClearGoal(start, goal);
  	return false;
  } 
  
  // Make sure the goal is in bounds
  if( !parent_->path_checker_->isPoseClear(goal) )
  {
  	ROS_INFO("[planning] Goal pose blocked!");
  	return false;
  } 
   
  ros::Time t1 = ros::Time::now();
	bool success = lattice_planner_->makeSegmentPlan(start, goal, path);
	ros::Duration t = ros::Time::now() - t1;
	
	
	if( success == false ){
		ROS_INFO("[planning] Point-point plan failed! (took %.2f seconds)",t.toSec());
		return false;
	}
	
	ROS_INFO("[planning] Point-point plan made in %.2f seconds. Plan has %lu segments.", t.toSec(), path.segs.size());
	
	return true;
}
 
bool
BKPlanningThread::planApproximateToGoal(const PoseStamped& start,
                                 const PoseStamped& goal,
                                 p_nav::Path&       path)
{
	// Modify the goal's angle
	double dx = goal.pose.position.x - start.pose.position.x;
	double dy = goal.pose.position.y - start.pose.position.y;
	PoseStamped newgoal = goal;
	newgoal.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(dy,dx));
	
	// Generate potential goal poses
	vector<PoseStamped> cleared_goals = generatePotentialGoals(newgoal);
	
  if( cleared_goals.size() == 0 ) {
  	ROS_INFO("[planning] All potential goal poses blocked!");
  	return false;
  } 
	
	bool found_goal = false;
	for( int igoal=0; igoal<cleared_goals.size(); igoal++ )
	{
		found_goal = planPointToPoint(start, cleared_goals.at(igoal), path);
		
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

void
BKPlanningThread::commitPathSegments()
{
	shared_ptr<BKFeederThread> feeder_(feeder_weak_);
				
	recursive_mutex::scoped_try_lock l(committed_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	p_nav::PathSegment seg_just_committed;
	double dist_just_committed;
	
	// How much path the feeder has left
	double dist_left = feeder_->getFeederDistLeft();
	
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
BKPlanningThread::commitOneSegment()
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
BKPlanningThread::generatePotentialGoals(const PoseStamped& true_goal)
{
	shared_ptr<BKPlanner> parent_(parent_weak_);
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
	potential_goals.push_back(getPoseOffset(true_goal,  .75*pi, ds));
	potential_goals.push_back(getPoseOffset(true_goal, -.75*pi, ds));
	
	// Add a goal 180 degrees offset
	potential_goals.push_back(getPoseOffset(true_goal, 1.00*pi, ds));
	
	// Eliminate goals in collision
	vector<PoseStamped> cleared_goals = parent_->path_checker_->getGoodPoses(potential_goals);
	
	// Store the candidate goals for visualization
	pub_goals_.poses.clear();
	pub_goals_.header = true_goal.header;
	for(int i=0; i<cleared_goals.size(); i++){
		pub_goals_.poses.push_back(cleared_goals.at(i).pose);
	}
	
	return cleared_goals;
}

bool
BKPlanningThread::segmentsAvailable()
{
	recursive_mutex::scoped_try_lock l(committed_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
		
	int num_available = committed_path_.segs.size();
	return num_available > 0;
}

precision_navigation_msgs::Path
BKPlanningThread::dequeueSegments()
{
	recursive_mutex::scoped_try_lock l(committed_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	// Return the current committed path, and clear it.
	precision_navigation_msgs::Path segs_to_return = committed_path_;
	committed_path_.segs.clear();
	
	return segs_to_return;
}

void
BKPlanningThread::enqueueSegments(precision_navigation_msgs::Path new_segments)
{
	recursive_mutex::scoped_try_lock l(committed_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
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
BKPlanningThread::escalatePlannerState(plannerState newstate)
{
	recursive_mutex::scoped_try_lock l(planner_state_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(planner_state_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	if( newstate > planner_state_ )
		planner_state_ = newstate;
}

void 
BKPlanningThread::setPlannerState(plannerState newstate)
{
	recursive_mutex::scoped_try_lock l(planner_state_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(planner_state_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	planner_state_ = newstate;
}

plannerState 
BKPlanningThread::getPlannerState()
{
	recursive_mutex::scoped_try_lock l(planner_state_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(planner_state_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	return planner_state_;
}
};//namespace
