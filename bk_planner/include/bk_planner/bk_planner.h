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

	// shorthand
	namespace p_nav   = precision_navigation_msgs;
	namespace bk_sbpl = bk_sbpl_lattice_planner;
	
	using     geometry_msgs::PoseStamped;
	using     std::vector;
	using     segment_lib::pi;
	
	// Declare these classes so we can use their names in BKPlanner
	class BKFeederThread;
	class BKPlanningThread;
	
	class BKPlanner
	{
	public:
		BKPlanner(std::string name, tf::TransformListener& tf);
		~BKPlanner();
		ros::NodeHandle  nh_;
		ros::NodeHandle  priv_nh_;
		
		// gotNewGoal is true if we recieved a new goal callback since the last time getLatestGoal() was called
		bool gotNewGoal();
    PoseStamped getLatestGoal();
    
		boost::shared_ptr<costmap_2d::Costmap2DROS>  planner_costmap_;
		boost::shared_ptr<path_checker::PathChecker> path_checker_;
		boost::recursive_mutex                       path_checker_mutex;
		
    boost::shared_ptr<BKPlanningThread> planner_;
    boost::shared_ptr<BKFeederThread>   feeder_;
    
	private:
		boost::shared_ptr<boost::thread> planning_thread_, feeder_thread_;
		void terminateThreads();
		
		ros::Subscriber        goal_sub_;
		tf::TransformListener& tf_;
		
		// Main thread handles all ROS callbacks
		void goalCB(const PoseStamped::ConstPtr& goal);
		PoseStamped poseToGlobalFrame(const PoseStamped& pose_msg);
		
		// Main thread sets the goal in a callback, planning thread checks it.
		// These functions are thread-safe.
    void setNewGoal(PoseStamped new_goal);
    
    // Do not directly access these variables, not thread-safe.
		bool                       got_new_goal_;
		PoseStamped latest_goal_;
		boost::recursive_mutex     goal_mutex_;
	};//BKPlanner
	
	enum plannerState
	{
		GOOD                 = 0,
		NEED_PARTIAL_REPLAN  = 1,
		NEED_FULL_REPLAN     = 2,
		NEED_RECOVERY        = 3,
		IN_RECOVERY          = 4
	};
	
	class BKPlanningThread
	{
	public:
		BKPlanningThread(BKPlanner* parent);
	
		void run();
		
		// Escalates to a higher need of replanning/recovery
		void escalatePlannerState(plannerState newstate);
		void setPlannerState(plannerState newstate);
		plannerState getPlannerState();
		
		// Retrieve the path segments that the planning thread has committed to steering
		bool segmentsAvailable();
		precision_navigation_msgs::Path dequeueSegments();
		// You can grab the mutex if you are making multiple calls
		boost::recursive_mutex          committed_path_mutex_;
	
	private:
		BKPlanner* parent_;

		// The distance of the commited path that the planner will try to maintain at all times (meters)
		double commit_distance_;
		// If planner can not get exactly to the target, it will try to get this far away
		double standoff_distance_;
		
		// Manage planning
    void doRecovery();
		void startRecovery();
    bool doFullReplan();
    bool doPartialReplan();
    void commitPathSegments();
		void enqueueSegments(precision_navigation_msgs::Path new_segments);
		p_nav::PathSegment commitOneSegment();
		
		// Makes a plan from start to goal
		bool planPointToPoint(const PoseStamped& start,
					                const PoseStamped& goal,
					                precision_navigation_msgs::Path&  segment_plan);
		
		// Makes a plan from start to as near as possible to goal
		bool planApproximateToGoal(const PoseStamped& start,
		                           const PoseStamped& goal,
		                           p_nav::Path&       path);
                               
                               
		PoseStamped getPoseOffset(const PoseStamped& pose, double dtheta, double dx);

		// Generates candidate goals centered on the true goal
		vector<PoseStamped> generatePotentialGoals(const PoseStamped& true_goal);
		
		p_nav::Path planner_path_;
		PoseStamped last_committed_pose_;
		int         last_committed_segnum_;
		
		boost::shared_ptr<bk_sbpl::BKSBPLLatticePlanner>  lattice_planner_;
		boost::shared_ptr<segment_lib::SegmentVisualizer> visualizer_;

		// Save a copy of the goals for visualization
		ros::Publisher candidate_goal_pub_;
		geometry_msgs::PoseArray pub_goals_;
		
    // Do not directly access these variables, not thread-safe.
		plannerState                    planner_state_;
		boost::recursive_mutex          planner_state_mutex_;
		precision_navigation_msgs::Path committed_path_;
	};//BKPlanningThread
	
	class BKFeederThread
	{
	public:	
		BKFeederThread(BKPlanner* parent);
	
		void run();
	
		// Thread-safe functions for managing the state of the feeder
		void   setFeederEnabled(bool state);
		void   sendResetSignals();
		double getFeederDistLeft();
		
		// A lock on this will prevent the feeder from running its main loop
		boost::recursive_mutex  feeder_lock_mutex_;
		
	private:
		BKPlanner* parent_;
		
		// Keep around this many already-completed segments for show
		int    segs_to_trail_;
		
		bool isFeederEnabled();
		void sendHaltState();
		void getNewSegments();
		void updatePathVelocities();
		bool hasProgressBeenMade();
		void resetStuckTimer();
		bool isStuckTimerFull();
		bool isPathClear();

		void executePath();
		void discardOldSegs();
		
		void doneCb(const actionlib::SimpleClientGoalState& state,
                const precision_navigation_msgs::ExecutePathResultConstPtr& result);
		void activeCb();
		void feedbackCb(const precision_navigation_msgs::ExecutePathFeedbackConstPtr& feedback);
		
		precision_navigation_msgs::ExecutePathFeedback latest_feedback_;
		boost::mutex                                   feedback_mutex_;
		precision_navigation_msgs::Path                feeder_path_;
		bool                                           feeder_path_has_changed_;
		
		boost::shared_ptr<segment_lib::SegmentVisualizer> visualizer_;
		actionlib::SimpleActionClient<precision_navigation_msgs::ExecutePathAction> client_;
		bool client_has_goal_;
		
		// Do not directly access these variables, not thread-safe.
		bool feeder_enabled_;
		boost::recursive_mutex feeder_enabled_mutex_, feeder_path_mutex_;
	};//BKFeederThread
};//namespace
#endif
