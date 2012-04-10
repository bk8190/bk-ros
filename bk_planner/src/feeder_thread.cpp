#include <bk_planner/bk_planner.h>

namespace bk_planner {

BKFeederThread::BKFeederThread(BKPlanner* parent):
	parent_ (parent),
	client_ ("execute_path", true)
{
	parent_->priv_nh_.param("planning/segs_to_trail"   , segs_to_trail_ , 4);
	parent_->priv_nh_.param("planning/feeder_loop_rate", loop_rate_     , 5.0);
	ROS_INFO("[feeder] Trailing segs:  %d", segs_to_trail_);
	
	visualizer_ = shared_ptr<segment_lib::SegmentVisualizer>
		(new segment_lib::SegmentVisualizer(std::string("feeder_visualization")) );

	feeder_path_.segs.clear();
	feeder_enabled_        = false;
	client_has_goal_       = false;
	latest_feedback_.current_segment.seg_number = 0;
	latest_feedback_.seg_distance_done = 0;0;
	
	ROS_INFO("Waiting for action server...");
	client_.waitForServer();
	
	ROS_INFO("[feeder] Constructor finished");
}

void
BKFeederThread::setPlanner(boost::weak_ptr<BKPlanningThread> planner)
{
	planner_ = planner;
}

void
BKFeederThread::run()
{	
	//ROS_INFO("[feeder] Main thread started at %.2fHz", loop_rate_);
	ros::Rate r(loop_rate_); // hz
	
	while(ros::ok())
	{
		{
			// Wait until we are allowed to run the feeder
			recursive_mutex::scoped_try_lock l(feeder_lock_mutex_);
		
			if(!l){
				boost::this_thread::interruption_point();
				ROS_WARN_THROTTLE(2,"[feeder] Locked out!");
				sendHaltState();
				r.sleep();
				continue;
			}
		
			// If we are not enabled, clear our path and send a halt to steering.
			if( !isFeederEnabled() )
			{
				ROS_INFO_THROTTLE(5,"[feeder] Feeder disabled");
				boost::this_thread::interruption_point();
				feeder_path_.segs.clear();
				sendHaltState();
			}
			else
			{
				// Get new segments from the planner, and drop old, already-traveled segments
				getNewSegments();
				discardOldSegs();
				
				// Find safe velocities
				updatePathVelocities();
			
				// Request a replan if the robot has not moved in a while
				if( hasProgressBeenMade() )
				{
					resetStuckTimer();
				}
				else if(isStuckTimerFull())
				{
					ROS_INFO("[feeder] Stuck timer full, requesting replan.");
					PlanThreadPtr(planner_)->escalatePlannerState(NEED_FULL_REPLAN);
				}
			
				// Check if the path is clear.  If so, send it to precision steering.
				if( parent_->path_checker_->isPathClear(feeder_path_) )
				{
					//ROS_INFO_THROTTLE(5,"[feeder] Executing path.");
					ROS_DEBUG_STREAM("[feeder] I have " << getFeederDistLeft() << "m left");
					executePath();
				}
				else
				{
					ROS_INFO("[feeder] Path blocked, requesting replan.");
					setFeederEnabled(false); // lololl i lock myself out
					PlanThreadPtr(planner_)->escalatePlannerState(NEED_FULL_REPLAN);
				}
			}
		
			// We are planning in the odometry frame, which constantly is shifting.  Lie and say the plan was created right now to avoid using an old transform.
			if( feeder_path_.segs.size() > 0 ){
				feeder_path_.segs.back().header.stamp = ros::Time::now();
				segment_lib::reFrame(feeder_path_);
			}
			// Have the visualizer publish visualization
			visualizer_->publishVisualization(feeder_path_);
		}
		
		boost::this_thread::interruption_point();
		r.sleep();
		boost::this_thread::interruption_point();
	}
}

// Cancel any goal in progress
void
BKFeederThread::sendHaltState()
{
	ROS_INFO("[feeder] Sending halt state");

	// Clear our path			
	recursive_mutex::scoped_try_lock l(feeder_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(feeder_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	feeder_path_.segs.clear();
			
	// If we've given steering a goal, clear it and send it a new trivial goal (holding position)	
	//if(client_has_goal_ == true)
	{
		PoseStamped pose;
		if(!parent_->getRobotPose(pose)) {
			ROS_ERROR("[feeder] Couldn't get robot pose to make halt state");
		}
			
		// Make an arbitrary path segment with no speed
		p_nav::PathSegment seg;
		seg.header.frame_id      = "/odom";
		seg.header.stamp         = ros::Time::now();
		seg.seg_number           = 0;
		seg.max_speeds.linear.x  = 0.0;
		seg.max_speeds.angular.z = 0.0;
		seg.min_speeds.linear.x  = 0.0;
		seg.min_speeds.angular.z = 0.0;
		seg.accel_limit          = 0.0;
		seg.decel_limit          = 0.0;
		seg.seg_type       = p_nav::PathSegment::LINE;
		seg.seg_length     = 0.01;
		seg.ref_point.x    = pose.pose.position.x;
		seg.ref_point.y    = pose.pose.position.y;
		seg.ref_point.z    = 0.0;
		seg.init_tan_angle = pose.pose.orientation;
	
		p_nav::ExecutePathGoal action_goal;
		action_goal.segments.clear();
		action_goal.segments.push_back(seg);
	
		client_.sendGoal(action_goal,
				boost::bind(&BKFeederThread::doneCb    , this, _1, _2),
				boost::bind(&BKFeederThread::activeCb  , this        ),
				boost::bind(&BKFeederThread::feedbackCb, this, _1    ));
		client_has_goal_ = false;
	}
}

void
BKFeederThread::getNewSegments()
{
	recursive_mutex::scoped_try_lock l1(PlanThreadPtr(planner_)->committed_path_mutex_);
	recursive_mutex::scoped_try_lock l2(feeder_path_mutex_);
	
	while(!l1){
		l1 = recursive_mutex::scoped_try_lock(PlanThreadPtr(planner_)->committed_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	while(!l2){
		l2 = recursive_mutex::scoped_try_lock(feeder_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	if( PlanThreadPtr(planner_)->segmentsAvailable() )
	{
		p_nav::Path new_segs = PlanThreadPtr(planner_)->dequeueSegments();
	
		// Make sure we actually got segments
		if(new_segs.segs.size() == 0){
			ROS_WARN("[feeder] Got new path but it was empty. WTF?");
			return;
		}
		
		feeder_path_has_changed_ = true;
		
		// No current path exists - get the new segments
		if( feeder_path_.segs.size() == 0 ) {
			ROS_INFO("[feeder] Got new path, %llu segs, first number %d", new_segs.segs.size(), new_segs.segs.front().seg_number);
			feeder_path_ = new_segs;
			latest_feedback_.seg_number = new_segs.segs.front().seg_number; // Hacky
			return;
		}
	
		//ROS_INFO("[feeder] Previous end %d, new front %d",
		//         feeder_path_.segs.back().seg_number, new_segs.segs.front().seg_number);
			         
		// A path already exists.  Check if the segment numbers are continuous
		if(feeder_path_.segs.back().seg_number+1 != new_segs.segs.front().seg_number){
			ROS_INFO("[feeder] Got discontinuous segments. Flushing extant path.");
			feeder_path_ = new_segs;
			return;
		}
	
		// Append new segments to current ones
		//ROS_INFO("[feeder] Appending %u new segments (%d to %d)", new_segs.segs.size(), new_segs.segs.front().seg_number, new_segs.segs.back().seg_number);
		
		feeder_path_.segs.insert(feeder_path_.segs.end(), new_segs.segs.begin(), new_segs.segs.end());
	}
}

void
BKFeederThread::updatePathVelocities()
{
	recursive_mutex::scoped_try_lock l(feeder_path_mutex_);
	
	while(!l){
		l = recursive_mutex::scoped_try_lock(feeder_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	// Get safe velocities for the segments
	parent_->path_checker_->assignPathVelocity(feeder_path_);
}

bool
BKFeederThread::hasProgressBeenMade()
{
	return true;
}

void
BKFeederThread::resetStuckTimer()
{
	return;
}

bool
BKFeederThread::isStuckTimerFull()
{
	return false;
}

bool
BKFeederThread::isPathClear()
{
	return parent_->path_checker_->isPathClear(feeder_path_);
}

void
BKFeederThread::executePath()
{
	recursive_mutex::scoped_try_lock l(feeder_path_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(feeder_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	// If we have a plan to execute, and it has changed since we last sent an execute request to steering
	if( feeder_path_.segs.size() > 0 && feeder_path_has_changed_)
	{
		// We have now handled the whole path.
		feeder_path_has_changed_ = false;
		
		//ROS_INFO("[feeder] Executing path");
		
		// Format the plan into a goal message and send it to the server.
		client_.waitForServer();
		p_nav::ExecutePathGoal action_goal;
		action_goal.segments = feeder_path_.segs;
		client_.sendGoal(action_goal,
				boost::bind(&BKFeederThread::doneCb    , this, _1, _2),
				boost::bind(&BKFeederThread::activeCb  , this        ),
				boost::bind(&BKFeederThread::feedbackCb, this, _1    ));
		client_has_goal_ = true;
	}
	else
	{
		//ROS_INFO_THROTTLE(2,"[feeder] No path");
	}
}

// Called once when the goal completes
void
BKFeederThread::doneCb(const actionlib::SimpleClientGoalState& state,
                  const p_nav::ExecutePathResultConstPtr& result)
{
  ROS_INFO("[feeder] Steering finished in state [%s]", state.toString().c_str());
}

// Called once when the goal becomes active
void
BKFeederThread::activeCb()
{
  //ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void
BKFeederThread::feedbackCb(const p_nav::ExecutePathFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback. Seg number %u, current seg %u, dist done %.2f", feedback->seg_number, feedback->current_segment.seg_number, feedback->seg_distance_done);

	// Try to get an exclusive lock on the feedback object.
	// Feedback arrives rapidly, so don't worry if we miss one every so often (hence the try lock)
	mutex::scoped_try_lock l(feedback_mutex_);
	if( l ){
	  latest_feedback_ = *feedback;
	}
}

void
BKFeederThread::discardOldSegs()
{
	// We don't want new feedback to arrive while we are in the middle of using it
	mutex::scoped_try_lock           l1(feedback_mutex_);
	recursive_mutex::scoped_try_lock l2(feeder_path_mutex_);
	while(!l1){
		l1 = mutex::scoped_try_lock(feedback_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	while(!l2){
		l2 = recursive_mutex::scoped_try_lock(feeder_path_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	int curr_seg_number = latest_feedback_.current_segment.seg_number;
	
	if(feeder_path_.segs.size() > 0)
	{
		//ROS_INFO("Active seg %d, front %d", curr_seg_number, feeder_path_.segs.front().seg_number);
	
		// Drop old segments ( but keep some trailing )
		// TODO: hacky logic makes bunny cry
		while( feeder_path_.segs.size() > 0 
		    && curr_seg_number > feeder_path_.segs.front().seg_number
		    && curr_seg_number-feeder_path_.segs.front().seg_number > segs_to_trail_ )
		{
			//ROS_INFO("[feeder] Dropping segment %d", feeder_path_.segs.front().seg_number);
			feeder_path_.segs.erase( feeder_path_.segs.begin() );
		}
	}
}


void
BKFeederThread::setFeederEnabled(bool state)
{
	recursive_mutex::scoped_try_lock l(feeder_enabled_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(feeder_lock_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	feeder_enabled_ = state;
}

bool
BKFeederThread::isFeederEnabled()
{
	recursive_mutex::scoped_try_lock l(feeder_enabled_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(feeder_enabled_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	return feeder_enabled_;
}

void 
BKFeederThread::sendResetSignals()
{
	// Wait for the feeder to catch up and finish its main loop
	recursive_mutex::scoped_try_lock l(feeder_lock_mutex_);
	while(!l){
		l = recursive_mutex::scoped_try_lock(feeder_lock_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	// Disable the feeder
	setFeederEnabled(false);
}

// Returns the a snapshot of linear distance left for the feeder
double
BKFeederThread::getFeederDistLeft()
{
	p_nav::Path p;
	int current_segnum;
	double current_seg_complete = 0.0;
	double d = 0.0;
	
	{
		recursive_mutex::scoped_try_lock l(feeder_path_mutex_);
		while(!l){
			l = recursive_mutex::scoped_try_lock(feeder_path_mutex_);
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		}
		
		p = feeder_path_;
	}
	
	{
		mutex::scoped_try_lock l(feedback_mutex_);
		while(!l){
			l = mutex::scoped_try_lock(feedback_mutex_);
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		}
		
		current_segnum       = latest_feedback_.current_segment.seg_number;
		current_seg_complete = latest_feedback_.seg_distance_done;
	}
	
	if( p.segs.size() == 0 ){
		return 0.0;
	}
	
	int start_idx = segment_lib::segnumToIndex(p, current_segnum);
	
	if( start_idx == -1 ){
		return 0.0;
	}
	

	// Length of first segment
	d = segment_lib::linDist(p.segs.at(start_idx));
	d = max(0.0, d-current_seg_complete);
	
	if( p.segs.at(start_idx).seg_type == p_nav::PathSegment::SPIN_IN_PLACE )
	{
		d *= 1.0;
	}
	
	// Length of the rest
	if( p.segs.size() > 1)
	{
		for( unsigned int i = start_idx + 1; i < p.segs.size(); i++ )
		{
			double d2 = segment_lib::linDist(p.segs.at(i));
			
			if( p.segs.at(i).seg_type == p_nav::PathSegment::SPIN_IN_PLACE )
			{
				d2 *= 1.0;
			}
			d += d2;
		}
	}
	
	return d;
}

};//namespace
