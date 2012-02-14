#ifndef BK_PLANNER_H_
#define BK_PLANNER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <bk_sbpl_lattice_planner/bk_sbpl_lattice_planner.h>

#include <precision_navigation_msgs/Path.h>
#include <precision_navigation_msgs/PathSegment.h>
#include <tf/transform_listener.h>

namespace bk_planner {

	class BKPlanner
	{
		public:
			BKPlanner(std::string name, tf::TransformListener& tf);
			~BKPlanner();
			
			//void initialize(std::string name, 
	
		private:
      costmap_2d::Costmap2DROS planner_costmap_ros_;
      bk_sbpl_lattice_planner::BKSBPLLatticePlanner lattice_planner_;
      
      ros::NodeHandle nh_;
			ros::Subscriber goal_sub_;
			ros::Publisher  plan_pub_;
			ros::Publisher  plan_vis_pub_;
  		tf::TransformListener& tf_;
  		
  		double test_param_;
			
			void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
			bool makePlan(const geometry_msgs::PoseStamped& goal);
			
			
			//boost::recursive_mutex configuration_mutex_;
			//dynamic_reconfigure::Server<bk_planner::BKPlannerConfig> *dsrv_;
			//void BKPlanner::reconfigureCB(bk_planner::BKPlannerConfig &config, uint32_t level);
	};



}; //namespace
#endif
