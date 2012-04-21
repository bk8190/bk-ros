#include <bk_planner/path_checker.h>

namespace path_checker
{

PathChecker::PathChecker(std::string name, boost::shared_ptr<costmap_2d::Costmap2DROS> costmap):
	costmap_(costmap),
	private_nh_("~/"+name)
{
	private_nh_.param("speed_lim/max_spd_x" , max_speed_.linear .x, 0.1);
	private_nh_.param("speed_lim/max_spd_th", max_speed_.angular.z, 0.1);
	
	private_nh_.param("accel_lim/max_acc_x" , max_accel_.linear .x, 0.0);
	private_nh_.param("accel_lim/max_acc_y" , max_accel_.linear .y, 0.0);
	private_nh_.param("accel_lim/max_acc_th", max_accel_.angular.z, 0.0);
	
	private_nh_.param("interpolation/dx"      , interp_dx_     , 0.01);
	private_nh_.param("interpolation/dth"     , interp_dth_    , 3.141/32.0);

	ROS_INFO("[path checker] Got max speed (x,th)  =(%.2f,%.2f)", max_speed_.linear.x, max_speed_.angular.z);
	ROS_INFO("[path checker] Got max accel (x,y,th)=(%.2f,%.2f,%.2f)", max_accel_.linear.x, max_accel_.linear.y, max_accel_.angular.z);
	ROS_INFO("[path checker] dx= %.2f dth=%.2f", interp_dx_, interp_dth_);
}


// Fills in a safe velocity to a path based on nearby obstacles
void
PathChecker::assignPathVelocity(p_nav::Path& path)
{
	// Treat each segment independently
	for( unsigned int i=0; i<path.segs.size(); i++ )
	{
		assignSegVelocity(path.segs.at(i));
	}
}


// Fills in a safe velocity to a segment based on nearby obstacles
void
PathChecker::assignSegVelocity(p_nav::PathSegment& seg)
{
	// First time processing this segment, speed will be zero
	if( seg.max_speeds.linear.x == 0 )
	{
		seg.max_speeds.linear.x  = max_speed_.linear.x;
		seg.max_speeds.angular.z = max_speed_.angular.z;
	}
	
	switch(seg.seg_type)
	{
		case p_nav::PathSegment::LINE:
			seg.accel_limit = max_accel_.linear.x;
			seg.decel_limit = max_accel_.linear.x;
			
			// HACK: If the length is negative, set it positive but set the max speed negative
			if( seg.seg_length < 0 ){
				//ROS_INFO("[path_checker] Found reverse segment");
				seg.seg_length = fabs(seg.seg_length);
				seg.max_speeds.linear.x *= -1.0;
			}
			
			break;
			
		case p_nav::PathSegment::ARC:
			seg.accel_limit = max_accel_.linear.x;
			seg.decel_limit = max_accel_.linear.x;
		break;
		
		case p_nav::PathSegment::SPIN_IN_PLACE:
			seg.accel_limit = max_accel_.angular.z;
			seg.decel_limit = max_accel_.angular.z;
		break;
	}
}

bool
PathChecker::isPoseClear(const PoseStamped& pose) {
	return isPoseClear(pose, 1.0);
}

bool
PathChecker::isPoseClear(const PoseStamped& pose, double alpha)
{
	// Create a copy of the costmap
	costmap_2d::Costmap2D map;
	costmap_->getCostmapCopy(map);
	base_local_planner::CostmapModel model(map);
	
	return isPoseClear(model, pose, alpha);
}

bool
PathChecker::isPoseClear(base_local_planner::CostmapModel& model, const PoseStamped& pose) {
	return isPoseClear(model, pose, 1.0);
}

bool
PathChecker::isPoseClear(base_local_planner::CostmapModel& model, const PoseStamped& pose, double alpha)
{
	if( alpha < 0.0 || alpha > 1.0 ){
		ROS_WARN("[path_checker] Invalid value of alpha (%.2f), it must be 0<x<1", alpha);
		alpha = std::min(alpha,1.0);
		alpha = std::max(alpha,0.0);
	}

	if( pose.header.frame_id.compare(costmap_->getGlobalFrameID()) != 0 )
	{
		ROS_WARN("[path_checker] Tried to compare poses in different frames (\"%s\", \"%s\")", pose.header.frame_id.c_str(), costmap_->getGlobalFrameID().c_str());
		return false;
	}
	
	geometry_msgs::Point point;
	point.x = pose.pose.position.x;
	point.y = pose.pose.position.y;
	point.z = 0;
	
	// Get the oriented footprint of the robot at "pose"
	std::vector<geometry_msgs::Point> oriented_footprint;
	costmap_->getOrientedFootprint(point.x, point.y, tf::getYaw(pose.pose.orientation), oriented_footprint);
	
	// Shrink the footprint to alpha times its original size
	for( unsigned int i=0; i<oriented_footprint.size(); i++ )
	{
		oriented_footprint.at(i).x = oriented_footprint.at(i).x*alpha + point.x*(1-alpha);
		oriented_footprint.at(i).y = oriented_footprint.at(i).y*alpha + point.y*(1-alpha);
	}
	
	double cost = model.footprintCost(point, oriented_footprint, 0.0, 0.0);
	//ROS_INFO("Pose has cost %.2f", cost);
	
	return  cost > -0.01;
}

vector<PoseStamped>
PathChecker::getGoodPoses(const vector<PoseStamped>& poses)
{
	// Create a copy of the costmap
	costmap_2d::Costmap2D map;
	costmap_->getCostmapCopy(map);
	base_local_planner::CostmapModel model(map);
	
	// Examine the poses one by one
	vector<PoseStamped> cleared;
	for( unsigned int ipose = 0; ipose<poses.size(); ipose++ )
	{
		if( isPoseClear(model, poses.at(ipose)) ) {
			cleared.push_back(poses.at(ipose));
		}
	}
	
	ROS_INFO("[path checker] %lu/%lu poses cleared.", cleared.size(), poses.size());
	return cleared;
}

// Returns true if nothing is blocking the segment
bool
PathChecker::isSegClear(const p_nav::PathSegment& seg)
{
	std::vector<PoseStamped> interp;
	interp = segment_lib::interpSegment(seg, interp_dx_, interp_dth_);
	bool clear = true;
	
	// Check points interpolated along the segment
	unsigned int i=0;
	for(i ; i<interp.size(); i++)
	{
		if( !isPoseClear(interp.at(i)) )
		{
			ROS_INFO("Obstacle at point %u on seg", i);
			clear = false;
			break;
		}
	}
	
	ROS_INFO("isSegClear: %u points checked", i+1);
	
	// No colisions found
	return clear;
}

// Returns the closest obstacle to the robot as its footprint moves along the segment
double
PathChecker::getClosestDist(const p_nav::PathSegment& seg)
{
	// All aboard the lulz boat
	return 9001;
}

// Returns indices of all segments colliding with obstacles
std::vector<unsigned int>
PathChecker::getBlockedSegs(p_nav::Path& path)
{
	std::vector<unsigned int> indices;
	indices.clear();
	
	for( unsigned int i=0; i<path.segs.size(); i++ ) {
		if( !isSegClear(path.segs.at(i)) ) {
			indices.push_back(i);
		}
	}
	return indices;
}
// Checks through the costmap, makes sure it doesn't run into any unknown/obstacle cells
bool
PathChecker::isPathClear(const p_nav::Path path)
{
	return isPathClear2(path);
	/*
	// Get a copy of the costmap
	costmap_2d::Costmap2D map;
	costmap_->getCostmapCopy(map);
	
	std::vector<PoseStamped> interp;
	geometry_msgs::Pose pose;
	double x, y;
	unsigned int x_c, y_c;
	bool inbounds;
	unsigned char cost;
	
	// Iterate over all segments
	for( unsigned int iseg=0; iseg<path.segs.size(); iseg++ )
	{
		interp = segment_lib::interpSegment(path.segs.at(iseg), interp_dx_, interp_dth_);
		
		// Iterate over all interpolated poses
		for( unsigned int ipose=0; ipose<interp.size(); ipose++ )
		{
				pose = interp.at(ipose).pose;
				x = pose.position.x;
				y = pose.position.y;
			
				// Coordinates in cells
				inbounds = map.worldToMap(x, y, x_c, y_c);
	
				if( !inbounds ) {
					ROS_INFO("[path checker] OOB point found at (%.2f,%.2f)", x, y);
					return false;
				}
	
				cost = map.getCost(x_c, y_c);
	
				if( cost > costmap_2d::INSCRIBED_INFLATED_OBSTACLE )
				{
					ROS_INFO("[path checker] Obstacle found at (%.2f,%.2f), value %hu", x, y, cost);
					return false;
				}
		
		}// pose
	}//segment
	
	return true;*/
}

// Checks through the costmap, makes sure it doesn't run into any unknown/obstacle cells
bool
PathChecker::isPathClear2(const p_nav::Path path)
{
	double alpha = 0.8;

	// Get a copy of the costmap
	costmap_2d::Costmap2D map;
	costmap_->getCostmapCopy(map);
	base_local_planner::CostmapModel model(map);
	
	std::vector<PoseStamped> interp;
	geometry_msgs::PoseStamped  pose;
	double x, y;
	
	// Iterate over all segments
	for( unsigned int iseg=0; iseg<path.segs.size(); iseg++ )
	{
		// Interpolate that segment and iterate over all poses
		interp = segment_lib::interpSegment(path.segs.at(iseg), interp_dx_, interp_dth_);
		for( unsigned int ipose=0; ipose<interp.size(); ipose++ )
		{
				pose = interp.at(ipose);
				x    = pose.pose.position.x;
				y    = pose.pose.position.y;
				
				if( !isPoseClear(model, pose, alpha) ) {
					ROS_INFO("[path checker] Obstacle found at (%.2f,%.2f) in segment %lu/%lu", x, y, iseg, path.segs.size());
					return false;
				}
		
		}// pose
	}//segment
	
	// No collisions found
	return true;
}

// Returns the lowest allowed velocity contained in the path
// Equivalent to min{ all_segs.max_vel }
double
PathChecker::getMinVelocity(const p_nav::Path path)
{
	// Really?
	return 100.0;
}

};//namespace
