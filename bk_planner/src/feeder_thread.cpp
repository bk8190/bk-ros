#include <bk_planner/bk_planner.h>

namespace bk_planner {

void BKPlanner::runFeederThread()
{
	long period_ms = (double)200 * 0.5; // 1/path_feeder_frequency_;
	ros::Rate r(5.0); // hz
	
	
	ROS_INFO("bk_planner path feeder thread started, period %ld", period_ms);
	
	feeder_visualizer_ = boost::shared_ptr<segment_lib::SegmentVisualizer>
		(new segment_lib::SegmentVisualizer(std::string("feeder_visualization")) );
	
	while(ros::ok())
	{
		// Wait until we are allowed to run the feeder
		boost::recursive_mutex::scoped_lock l(feeder_lock_mutex);
		
		if( !isFeederEnabled() )
		{
			ROS_INFO_THROTTLE(5,"[feeder] Feeder disabled");
			feeder_path_.segs.clear();
			sendHaltState();
		}
		else
		{
			// Get new segments from the planner
			getNewSegments();
		
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
				escalatePlannerState(NEED_PARTIAL_REPLAN);
			}
			
			discardOldSegs();
		
			// Check if the path is clear.  If so, send it to precision steering.
			if( isPathClear() ) // >0 velocity
			{
				executePath();
			}
			else
			{
				ROS_INFO("[feeder] Path blocked, requesting replan.");
				escalatePlannerState(NEED_FULL_REPLAN);
			}
		}
		
		// Have the visualizer publish visualization
		feeder_visualizer_->publishVisualization(feeder_path_);
		
		boost::this_thread::interruption_point();
		r.sleep();
	}
}

// Cancel any goal in progress
void
BKPlanner::sendHaltState()
{
	// Cancel any active goal
	if( (client_has_goal_ == true) && (client_.getState().isDone() == false) )
	{
		client_.stopTrackingGoal();
		client_.cancelGoal();
	}
}

void
BKPlanner::getNewSegments()
{
	boost::recursive_mutex::scoped_lock l1(committed_path_mutex_);
	boost::recursive_mutex::scoped_lock l2(feeder_path_mutex_);
	
	if( segmentsAvailable() )
	{
		precision_navigation_msgs::Path new_segs = dequeueSegments();
	
		// Make sure we actually got segments
		if(new_segs.segs.size() == 0){
			ROS_WARN("[feeder] Got new path but it was empty. WTF?");
			return;
		}
		
		feeder_path_has_changed_ = true;
		
		// No current path exists - get the new segments
		if( feeder_path_.segs.size() == 0 ) {
			ROS_INFO("[feeder] Got new path, %u segs, first number %d", new_segs.segs.size(), new_segs.segs.front().seg_number);
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
BKPlanner::updatePathVelocities()
{
	boost::recursive_mutex::scoped_lock l(feeder_path_mutex_);
	// Get safe velocities for the segments
	path_checker_->assignPathVelocity(feeder_path_);
}

bool
BKPlanner::hasProgressBeenMade()
{
	return true;
}

void
BKPlanner::resetStuckTimer()
{
	return;
}

bool
BKPlanner::isStuckTimerFull()
{
	return false;
}

bool
BKPlanner::isPathClear()
{
	return( path_checker_->isPathClear(feeder_path_) );
}

void
BKPlanner::executePath()
{
	boost::recursive_mutex::scoped_lock l(feeder_path_mutex_);
	
	// If we have a plan to execute, and it has changed since we last sent an execute request to steering
	if( feeder_path_.segs.size() > 0 && feeder_path_has_changed_)
	{
		// We have now handled the whole path.
		feeder_path_has_changed_ = false;
		
		//ROS_INFO("[feeder] Executing path");
		
		// Format the plan into a goal message and send it to the server.
		client_.waitForServer();
		precision_navigation_msgs::ExecutePathGoal action_goal;
		action_goal.segments = feeder_path_.segs;
		client_.sendGoal(action_goal,
				boost::bind(&BKPlanner::doneCb    , this, _1, _2),
				boost::bind(&BKPlanner::activeCb  , this        ),
				boost::bind(&BKPlanner::feedbackCb, this, _1    ));
		client_has_goal_ = true;
	}
	else
	{
		ROS_INFO_THROTTLE(2,"[feeder] No path");
	}
}

// Called once when the goal completes
void
BKPlanner::doneCb(const actionlib::SimpleClientGoalState& state,
                  const precision_navigation_msgs::ExecutePathResultConstPtr& result)
{
  ROS_INFO("[feeder] Steering finished in state [%s]", state.toString().c_str());
}

// Called once when the goal becomes active
void
BKPlanner::activeCb()
{
  //ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void
BKPlanner::feedbackCb(const precision_navigation_msgs::ExecutePathFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback. Seg number %u, current seg %u, dist done %.2f", feedback->seg_number, feedback->current_segment.seg_number, feedback->seg_distance_done);

	// Try to get an exclusive lock on the feedback object.
	// Feedback arrives rapidly, so don't worry if we miss one every so often (hence the try lock)
	boost::mutex::scoped_try_lock l(feedback_mutex_);
	if( l ){
	  latest_feedback_ = *feedback;
	}
}

void
BKPlanner::discardOldSegs()
{
	// We don't want new feedback to arrive while we are in the middle of using it
	boost::mutex::scoped_lock           l1(feedback_mutex_);
	boost::recursive_mutex::scoped_lock l2(feeder_path_mutex_);
	
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
};//namespace
