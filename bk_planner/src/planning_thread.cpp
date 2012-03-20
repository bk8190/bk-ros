#include <bk_planner/bk_planner.h>
#include "planning_lib.cpp"
namespace bk_planner {

BKPlanningThread::BKPlanningThread(BKPlanner* parent):
	parent_  (parent)
{
	;
	candidate_goal_pub_ = parent_->nh_.advertise<geometry_msgs::PoseArray>("candidate_poses", 1);

	// For visualizing the planning in progress
	visualizer_ = shared_ptr<segment_lib::SegmentVisualizer>
		(new segment_lib::SegmentVisualizer(std::string("planning_visualization")) );
		
	lattice_planner_ = shared_ptr<bk_sbpl::BKSBPLLatticePlanner>
		(new bk_sbpl::BKSBPLLatticePlanner("lattice_planner", parent_->planner_costmap_) );
	
	parent_->priv_nh_.param("planning/commit_distance"   , commit_distance_   ,1.1);
	parent_->priv_nh_.param("planning/standoff_distance" , standoff_distance_ ,1.1);
	parent_->priv_nh_.param("planning/short_distance"    , short_dist_        ,1.1);
	parent_->priv_nh_.param("planning/planner_loop_rate" , loop_rate_         ,1.0);
	parent_->priv_nh_.param("planning/override_goal_gen" , override_goal_generation_ ,false);
	ROS_INFO("[planning] Commit   distance %.2f", commit_distance_  );
	ROS_INFO("[planning] Standoff distance %.2f", standoff_distance_);
	ROS_INFO("[planning] Short    distance %.2f", short_dist_       );
	
	if( override_goal_generation_ ){
		ROS_WARN("[planning] Ignoring goal generation");
	}
	
	goal_pub_ = parent->nh_.advertise<PoseStamped>("target_goal", 1);
	
	planner_path_.segs.clear();
	planner_state_         = GOOD;
	last_committed_segnum_ = 0;
	last_committed_pose_   = parent_->getLatestGoal();
	last_scrapped_path_    = ros::Time::now();
	scrap_path_timeout_    = ros::Duration(5.0);
	
	ROS_INFO("[planning] Constructor finished");
}
	

void
BKPlanningThread::setFeeder(boost::weak_ptr<BKFeederThread> feeder)
{
	feeder_ = feeder;
}

void
BKPlanningThread::run()
{
	ROS_INFO("[planning] Main thread started at %.2fHz", loop_rate_);
	bool made_one_plan = false; // true if we made at least one plan
	bool path_clear, segs_left;
	ros::Rate r(loop_rate_); // hz
		
	while(ros::ok()) // Main loop
	{
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
				FeedThreadPtr(feeder_)->sendResetSignals(); // bring the system to a halt
			
				if( doFullReplan() ) {
					setPlannerState(GOOD);
					made_one_plan = true;
				}else
					escalatePlannerState(NEED_RECOVERY);
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
				if( !made_one_plan ){
					if( parent_->gotNewGoal()) {
						escalatePlannerState(NEED_PARTIAL_REPLAN);
					}
					break;
				}
				path_clear = parent_->path_checker_->isPathClear(planner_path_);
				segs_left  = planner_path_.segs.size()>0;
		
				// Commit path segments if the path is clear and we have a non-empty path
				if( path_clear && segs_left ) {
					ROS_INFO_THROTTLE(5, "[planning] Planner state good.");
					commitPathSegments();
					FeedThreadPtr(feeder_)->setFeederEnabled(true);
				}
				else if(segs_left) {
					// We found obstacles, signal a replan
					ROS_INFO_THROTTLE(1, "[planning] Found obstacles.");
					escalatePlannerState(NEED_PARTIAL_REPLAN);
				}
				else {
					ROS_INFO_THROTTLE(7, "[planning] Nothing more to pass down.");
				}
				
				// Check if there is a new goal. Note that this is done AFTER committing path segments.
				if( parent_->gotNewGoal()) {
					escalatePlannerState(NEED_PARTIAL_REPLAN);
				}
			break;	
			
			default:
				ROS_ERROR("[planning] Bad state = %d",s);
			break;
		}
	
		
		// We are planning in the odometry frame, which constantly is shifting.  Lie and say the plan was created right now to avoid using an old transform.
		if( planner_path_.segs.size() > 0 ){
			//planner_path_.segs.back().header.stamp = ros::Time::now();
			//segment_lib::reFrame(planner_path_);
			
			// Also reframe the candidate goals for visualization
			pub_goals_.header.stamp    = planner_path_.header.stamp;
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

// Starts recovery process, initializes recovery state variables
void
BKPlanningThread::startRecovery()
{
	ROS_INFO("[planning] Starting recovery");
	time_started_recovery_ = ros::Time::now();
	next_recovery_action_  = ros::Time::now();
	recovery_counter_      = 0;
}

void
BKPlanningThread::doRecovery()
{
	//ROS_INFO_THROTTLE(5,"[planning] In recovery");
	
	// TODO: Make a recovery counter/timer that will do several things in turn
	// (There is a replan attempt after each step, if it succeeds, recovery is done.)
	// 1: Clear obstacles within footprint from costmap and clear obstacles a bit behind
	// 2: Spin in place +- 15
	// 3: Back up
	// 4: Spin 360
	
	// If we are scheduled for a recovery action
	if( ros::Time::now() - next_recovery_action_ > ros::Duration(0) )
	{
		switch( recovery_counter_ )
		{
			// Clear obstacles from costmap
			case 0:
				ROS_INFO("[planning] Recovery: Clearing costmap");
				parent_->planner_costmap_->resetMapOutsideWindow(1.0,1.0);
				
				next_recovery_action_ = ros::Time::now() + ros::Duration(2.0);
				recovery_counter_++;
				break;
			
			// Back up
			case 1:
				recovery_counter_++;
				break;
			
			
			// TODO: the rest
			
			
			// Loop around
			default:
				recovery_counter_ = 0;
		}
		
		// Try to make a full replan.  If it succeeds, get out of recovery.
		if(parent_->gotNewGoal()) {
			if( doFullReplan() ) {
				ROS_INFO("[planning] Yay, we got out of recovery.");
				setPlannerState(GOOD);
			}
		}
	}
}

// TODO: Stop committing at a turn segment, and don't commit anything else until
void
BKPlanningThread::commitPathSegments()
{
	recursive_mutex::scoped_try_lock l(committed_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	// How much path the feeder has left
	double dist_left = FeedThreadPtr(feeder_)->getFeederDistLeft();
	
	// Desired distance to add
	double dist_to_add = commit_distance_ - dist_left;
	
	//ROS_INFO_THROTTLE(5,"[planning] Feeder has %.2f left, committing %.2f, I have %d segs left", dist_left, dist_to_add, planner_path_.segs.size());
	if( planner_path_.segs.size() > 0 && dist_to_add > 0 )
	{
		commitOneSegment();
	}
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

p_nav::Path
BKPlanningThread::dequeueSegments()
{
	recursive_mutex::scoped_try_lock l(committed_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	// Return the current committed path, and clear it.
	p_nav::Path segs_to_return = committed_path_;
	committed_path_.segs.clear();
	
	return segs_to_return;
}

void
BKPlanningThread::enqueueSegments(p_nav::Path new_segments)
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
