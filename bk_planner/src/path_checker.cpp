#include <bk_planner/path_checker.h>

namespace path_checker
{

PathChecker::PathChecker(std::string name, costmap_2d::Costmap2DROS* costmap):
	costmap_(costmap),
	private_nh_("~/"+name)
{
	private_nh_.param("speed_lim/max_spd_x" , max_speed_.linear .x, 0.0);
	private_nh_.param("speed_lim/max_spd_th", max_speed_.angular.z, 0.0);
	private_nh_.param("accel_lim/max_acc_x" , max_accel_.linear .x, 0.0);
	private_nh_.param("accel_lim/max_acc_y" , max_accel_.linear .y, 0.0);
	private_nh_.param("accel_lim/max_acc_th", max_accel_.angular.z, 0.0);

	ROS_INFO("Got max speed (x,th)  =(%.2f,%.2f)", max_speed_.linear.x, max_speed_.angular.z);
	ROS_INFO("Got max accel (x,y,th)=(%.2f,%.2f,%.2f)", max_accel_.linear.x, max_accel_.linear.y, max_accel_.angular.z);
}


// Fills in a safe velocity to a path based on nearby obstacles
void PathChecker::assignPathVelocity(precision_navigation_msgs::Path& path)
{
	// Treat each segment independently
	for( int i=0; i<path.segs.size(); i++ )
	{
		assignSegVelocity(path.segs.at(i));
	}
}


// Fills in a safe velocity to a segment based on nearby obstacles
void PathChecker::assignSegVelocity(precision_navigation_msgs::PathSegment& p)
{
	// herp de derp
	p.max_speeds.linear.x  = max_speed_.linear.x;
	p.max_speeds.angular.z = max_speed_.angular.z;
	
	switch(p.seg_type)
	{
		case precision_navigation_msgs::PathSegment::LINE:
			p.accel_limit = max_accel_.linear.x;
			p.decel_limit = max_accel_.linear.x;
			break;
			
		case precision_navigation_msgs::PathSegment::ARC:
			p.accel_limit = max_accel_.linear.x;
			p.decel_limit = max_accel_.linear.x;
		break;
		
		case precision_navigation_msgs::PathSegment::SPIN_IN_PLACE:
			p.accel_limit = max_accel_.angular.z;
			p.decel_limit = max_accel_.angular.z;
		break;
	}
}


// Returns true if nothing is blocking the segment
bool isSegClear(const precision_navigation_msgs::PathSegment& p)
{
	// herp de derp
	return true;
}

// Returns the closest obstacle to the robot as its footprint moves along the segment
double getClosestDist(const precision_navigation_msgs::PathSegment& p)
{
	// All aboard the lulz boat
	return 9001;
}

// Returns indices of all segments colliding with obstacles
std::vector<int> getBlockedSegs(precision_navigation_msgs::Path& path);
{
	std::vector<int> indices;
	indices.clear();
	
	for( int i=0; i<path.size(); i++ ) {
		if( !isSegClear(path.segs.at(i)) ) {
			indices.push_back(i);
		}
	}
	return indices;
}

};//namespace
