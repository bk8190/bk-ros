#ifndef PATH_CHECKER_H_
#define PATH_CHECKER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <segment_lib/segment_lib.h>

namespace path_checker{
	
	namespace p_nav = precision_navigation_msgs;
	using geometry_msgs::PoseStamped;
	using geometry_msgs::Pose;
	using std::vector;
	
	class PathChecker
	{
		public:
			PathChecker(std::string name, boost::shared_ptr<costmap_2d::Costmap2DROS> costmap);
			~PathChecker() {}

			// Fills in a safe velocity to a path based on nearby obstacles
			void assignPathVelocity(p_nav::Path& path);

			// Fills in a safe velocity to a segment based on nearby obstacles
			void assignSegVelocity(p_nav::PathSegment& seg);
			
			// Checks if a single pose is clear
			bool isPoseClear(const PoseStamped& pose);
			bool isPoseClear(const PoseStamped& pose, double alpha);
			bool isPoseClear(base_local_planner::CostmapModel& model, const PoseStamped& pose);
			bool isPoseClear(base_local_planner::CostmapModel& model, const PoseStamped& pose, double alpha);

			// Returns only the poses not in collision
			vector<PoseStamped> getGoodPoses(const vector<PoseStamped>& poses);

			// Returns true if nothing is blocking the segment
			bool isSegClear(const p_nav::PathSegment& seg);

			// Checks for obstacles along the path.  Returns a list of all colliding segment indices
			std::vector<unsigned int> getBlockedSegs(p_nav::Path& path);
			
			// Returns the closest obstacle to the robot as its footprint moves along the segment
			double getClosestDist(const p_nav::PathSegment& seg);
			
			// Checks through the costmap, makes sure it doesn't run into any unknown/obstacle cells
			bool isPathClear (const p_nav::Path path);
			bool isPathClear2(const p_nav::Path path);
			
			// Searches around the goal point for a valid pose
			bool getNearestClearGoal(const PoseStamped& start, const PoseStamped& oldgoal, PoseStamped& newgoal);
			
			// Returns the lowest allowed velocity contained in the path
			// Equivalent to min{ all_segs.max_vel }
			double getMinVelocity(const p_nav::Path path);
			
		private:
			boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
			ros::NodeHandle           private_nh_;

			// Absolute maximum speed/accel
			geometry_msgs::Twist max_speed_;
			geometry_msgs::Twist max_accel_;
			
			double interp_dx_, interp_dth_;

	};//class

};//namespace

#endif
