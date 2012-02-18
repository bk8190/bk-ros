#include <bk_planner/bk_planner.h>

namespace bk_planner {

void BKPlanner::runFeederThread()
{
	long period_ms = (double)1000 * 0.5; // 1/path_feeder_frequency_;
	
	ROS_INFO("bk_planner path feeder thread started, period %ld", period_ms);
	
	while(true)
	{
		if( isFeederEnabled() )
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
			
			// Check if the path is clear.  If so, send it to precision steering.
			if( isPathClear() ) // >0 velocity
			{
				ROS_INFO("[feeder] Executing path");
				executePath();
				
			}
		}
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(period_ms));
	}
}

void
BKPlanner::sendHaltState()
{

}

void
BKPlanner::getNewSegments()
{
	precision_navigation_msgs::Path new_segs = dequeueSegments();
	
	// No path exists - get the new segments
	if( feeder_path_.segs.size() == 0 ) {
		ROS_INFO("[feeder] Got new path");
		feeder_path_ = new_segs;
		return;
	}
	
	// A path already exists.  Check if the segment numbers are continuous
	if(committed_path_.segs.back().seg_number+1 != new_segs.segs.front().seg_number){
		ROS_INFO("[feeder] Got discontinuous segments. Flushing extant path.");
		feeder_path_ = new_segs;
		return;
	}
	
	ROS_INFO("[feeder] Appending new segments");
	// Append new segments to current ones
	feeder_path_.segs.insert(feeder_path_.segs.end(),
	                         new_segs.segs.begin(), new_segs.segs.end() );
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
	// Have the visualizer publish visualization
	segment_visualizer_->publishVisualization(feeder_path_);
	
	// Execute the whole plan
	client_.waitForServer();
	precision_navigation_msgs::ExecutePathGoal action_goal;
	action_goal.segments = feeder_path_.segs;
	client_.sendGoal(action_goal);
}


};//namespace
