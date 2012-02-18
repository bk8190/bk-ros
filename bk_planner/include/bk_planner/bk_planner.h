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
	enum plannerState
	{
		GOOD                 = 0,
		NEED_PARTIAL_REPLAN  = 1,
		NEED_FULL_REPLAN     = 2,
		NEED_RECOVERY        = 3,
		IN_RECOVERY          = 4
	};
			
	class BKPlanner
	{
		public:
			BKPlanner(std::string name, tf::TransformListener& tf);
			~BKPlanner();
			ros::NodeHandle  nh_;

		private:
			boost::shared_ptr<costmap_2d::Costmap2DROS>                      planner_costmap_;
			boost::shared_ptr<bk_sbpl_lattice_planner::BKSBPLLatticePlanner> lattice_planner_;
			boost::shared_ptr<path_checker::PathChecker>                     path_checker_;
			boost::shared_ptr<segment_lib::SegmentVisualizer>                segment_visualizer_;
			boost::shared_ptr<boost::thread> planning_thread_, feeder_thread_;

			ros::NodeHandle  priv_nh_;
			ros::Subscriber  goal_sub_;
			tf::TransformListener& tf_;

			double max_vel_x_;
			double max_rotational_vel_;
			double acc_lim_th_;
			double acc_lim_x_;
			double acc_lim_y_;
			
			void terminateThreads();

			// Main thread: goal callback
			void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
			geometry_msgs::PoseStamped poseToGlobalFrame(const geometry_msgs::PoseStamped& pose_msg);
			
			/* Common data and access functions for common data */
			/*============================================================*/
			void enableFeeder();
			void disableFeeder();
			
			void sendResetSignals();
			
			// Main thread sets the goal in a callback, planning thread checks it.
			// These functions are thread-safe.
      void setNewGoal(geometry_msgs::PoseStamped new_goal);
      bool gotNewGoal();
      geometry_msgs::PoseStamped getLatestGoal();
      
      // Do not directly access these variables, not thread-safe.
			bool                       got_new_goal_;
			geometry_msgs::PoseStamped latest_goal_;
			boost::recursive_mutex     goal_mutex_;

			// Thread-safe functions to indicate need for a replan or recovery

			
			// Escalates to a higher need of replanning/recovery
			void escalatePlannerState(plannerState newstate);
			
			// Sets the planner state
			void setPlannerState(plannerState newstate);
			plannerState getPlannerState();
			
      // Do not directly access these variables, not thread-safe.
			boost::recursive_mutex  planner_state_mutex_;
			plannerState            planner_state_;

			
			/* Planning thread exclusive functions and data */
			/*============================================================*/
			void runPlanningThread();
      void doRecovery();
			void startRecovery();
      bool doFullReplan();
      bool doPartialReplan();
      
      void commitPathSegments();
			bool planPointToPoint(const geometry_msgs::PoseStamped& start,
						                const geometry_msgs::PoseStamped& goal,
						                precision_navigation_msgs::Path&  segment_plan);
			
			
			/* Path feeder thread exclusive functions and data */
			/*============================================================*/
			void runFeederThread();
			int  last_committed_path_seg_;
			actionlib::SimpleActionClient<precision_navigation_msgs::ExecutePathAction> client_;
			
			//boost::recursive_mutex configuration_mutex_;
			//dynamic_reconfigure::Server<bk_planner::BKPlannerConfig> *dsrv_;
			//void BKPlanner::reconfigureCB(bk_planner::BKPlannerConfig &config, uint32_t level);

	};//class
};//namespace
#endif
