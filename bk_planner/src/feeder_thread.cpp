#include <bk_planner/bk_planner.h>

namespace bk_planner {

void BKPlanner::runFeederThread()
{
	long period_ms = (double)1000 * 0.5; // 1/path_feeder_frequency_;
	
	ROS_INFO("bk_planner path feeder thread started, period %ld", period_ms);
	
	while(true)
	{
	
		if( !isFeederEnabled() )
		{
			ROS_INFO("[feeder] Feeder disabled");
			sendHaltState();
			feeder_path_.segs.clear();
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
				escalatePlannerState(NEED_PARTIAL_REPLAN);
			}
		}
		
		// Have the visualizer publish visualization
		segment_visualizer_->publishVisualization(feeder_path_);
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(period_ms));
	}
}

void
BKPlanner::sendHaltState()
{
	return;
}

void
BKPlanner::getNewSegments()
{
	if( segmentsAvailable() )
	{
		feeder_path_has_changed_ = true;
		precision_navigation_msgs::Path new_segs = dequeueSegments();
	
		// No path exists - get the new segments
		if( feeder_path_.segs.size() == 0 ) {
			ROS_INFO("[feeder] Got new path, %d segs", new_segs.segs.size());
			feeder_path_ = new_segs;
			return;
		}
	
		// A path already exists.  Check if the segment numbers are continuous
		if(feeder_path_.segs.back().seg_number+1 != new_segs.segs.front().seg_number){
			ROS_INFO("[feeder] Got discontinuous segments. Flushing extant path.");
			feeder_path_ = new_segs;
			return;
		}
	
		ROS_INFO("[feeder] Appending %d new segments", new_segs.segs.size());
		// Append new segments to current ones
		feeder_path_.segs.insert(feeder_path_.segs.end(),
			                       new_segs.segs.begin(), new_segs.segs.end() );
	}
}

void
BKPlanner::updatePathVelocities()
{
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
	if( feeder_path_.segs.size() > 0 )
	{
		if( feeder_path_has_changed_ )
		{
			feeder_path_has_changed_ = false;
			ROS_INFO("[feeder] Executing path");
			// Execute the whole plan
			client_.waitForServer();
			precision_navigation_msgs::ExecutePathGoal action_goal;
			action_goal.segments = feeder_path_.segs;
			client_.sendGoal(action_goal,
					boost::bind(&BKPlanner::doneCb    , this, _1, _2),
					boost::bind(&BKPlanner::activeCb  , this        ),
					boost::bind(&BKPlanner::feedbackCb, this, _1    ));
		}
	}
	else
	{
		ROS_INFO("[feeder] No path");
	}
}

// Called once when the goal completes
void
BKPlanner::doneCb(const actionlib::SimpleClientGoalState& state,
                  const precision_navigation_msgs::ExecutePathResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void
BKPlanner::activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void
BKPlanner::feedbackCb(const precision_navigation_msgs::ExecutePathFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback. Seg number %u, current seg %u, dist done %.2f", feedback->seg_number, feedback->current_segment.seg_number, feedback->seg_distance_done);
  
	boost::recursive_mutex::scoped_lock l(feedback_mutex_);
  latest_feedback_ = *feedback;
}

void
BKPlanner::discardOldSegs()
{
	boost::recursive_mutex::scoped_lock l(feedback_mutex_);
	
	int curr_seg_number = latest_feedback_.current_segment.seg_number;
	
	if(feeder_path_.segs.size() > 0)
	{
		//ROS_INFO("Active seg (%d, curr %d) front %d", seg_number, curr_seg_number, feeder_path_.segs.at(0).seg_number);
	
		// Drop everything before the current segment
		while( feeder_path_.segs.front().seg_number < curr_seg_number && feeder_path_.segs.size() > 0)
		{
			feeder_path_.segs.erase( feeder_path_.segs.begin() );
		}
	}
}
};//namespace
