#ifndef BK_PLANNER_H_
#define BK_PLANNER_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>

#include <bk_sbpl_lattice_planner/bk_sbpl_lattice_planner.h>

#include <precision_navigation_msgs/Path.h>
#include <precision_navigation_msgs/PathSegment.h>
#include <precision_navigation_msgs/ExecutePathAction.h>
#include <precision_navigation_msgs/ExecutePathGoal.h>


#include <bk_planner/path_checker.h>
#include <segment_lib/segment_lib.h>
#include <segment_lib/SegmentVisualizer.h>

namespace bk_planner {

	class BKPlanner
	{
		public:
			BKPlanner(std::string name, tf::TransformListener& tf);
			~BKPlanner();
			
		private:
      boost::shared_ptr<costmap_2d::Costmap2DROS>                      planner_costmap_;
      boost::shared_ptr<bk_sbpl_lattice_planner::BKSBPLLatticePlanner> lattice_planner_;
      boost::shared_ptr<path_checker::PathChecker>                     path_checker_;
      boost::shared_ptr<segment_lib::SegmentVisualizer>                segment_visualizer_;
      
      boost::shared_ptr<boost::thread> planning_thread_;
      void planningThread();
      
      boost::shared_ptr<boost::thread> path_feeder_thread_;
      void pathFeederThread();
      
      
      // Every time a callback occurs with a new goal, this will be set to true.
			void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
      bool got_new_goal_;
      geometry_msgs::PoseStamped latest_goal_;
      
      int  last_committed_path_seg_;
      
      // We don't 
      boost::mutex                     planner_mutex_;
      
      ros::NodeHandle  nh_;
      ros::NodeHandle  priv_nh_;
			ros::Subscriber  goal_sub_;
			ros::Publisher   plan_pub_;
			ros::Publisher   plan_vis_pub_;
  		tf::TransformListener& tf_;
  		
  		actionlib::SimpleActionClient<precision_navigation_msgs::ExecutePathAction> client_;
  		
  		double max_vel_x_;
  		double max_rotational_vel_;
  		double acc_lim_th_;
  		double acc_lim_x_;
  		double acc_lim_y_;
  		
			
			bool makePlan(const geometry_msgs::PoseStamped& goal);
			
			geometry_msgs::PoseStamped poseToGlobalFrame(const geometry_msgs::PoseStamped& pose_msg);
			
			//boost::recursive_mutex configuration_mutex_;
			//dynamic_reconfigure::Server<bk_planner::BKPlannerConfig> *dsrv_;
			//void BKPlanner::reconfigureCB(bk_planner::BKPlannerConfig &config, uint32_t level);

	};//class
}; //namespace
#endif
