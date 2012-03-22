#ifndef BK_PLANNER_H_
#define BK_PLANNER_H_

#include <ros/ros.h>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
	
	using geometry_msgs::PoseStamped;
	using geometry_msgs::PoseWithCovarianceStamped;
	using std::vector;
	using segment_lib::pi;
	using boost::shared_ptr;
	using boost::recursive_mutex;
	using boost::mutex;
	
	// Declare these classes so we can use their names in BKPlanner, precious
	class BKFeederThread;
	class BKPlanningThread;
	
	typedef shared_ptr<BKFeederThread>   FeedThreadPtr;
	typedef shared_ptr<BKPlanningThread> PlanThreadPtr;
	
	double dist( const PoseStamped& p1, const PoseStamped& p2 );
	
	// Main class that handles a goal callbacks as well as managing a planning thread and a path feeder thread
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
    
		// Get the robot's current poseToGlobalFrame
		bool getRobotPose(PoseStamped& pose);
    
		shared_ptr<costmap_2d::Costmap2DROS>  planner_costmap_;
		shared_ptr<path_checker::PathChecker> path_checker_;
		recursive_mutex                       path_checker_mutex;
		
		tf::TransformListener& tf_;
		
	private:
    shared_ptr<BKPlanningThread> planner_;
    shared_ptr<BKFeederThread>   feeder_;
		shared_ptr<boost::thread>    planning_thread_, feeder_thread_;
		void terminateThreads();
		
		ros::Subscriber goal_sub_;
		ros::Duration   goal_timeout_;
		double  goal_cov_thresh_;
		
		// Main thread handles all ROS callbacks
		void goalCB(const PoseWithCovarianceStamped::ConstPtr& goal);
		bool poseToGlobalFrame(const PoseStamped& pose_msg, PoseStamped& transformed);
		
		// Main thread sets the goal in a callback, planning thread checks it.
		// These functions are thread-safe.
    void setNewGoal(PoseStamped new_goal);
    double goal_hysteresis_;
    
    // Do not directly access these variables, not thread-safe.
		bool            got_new_goal_;
		PoseStamped     latest_goal_;
		recursive_mutex goal_mutex_;
		
		ros::Time       last_accepted_goal_;
	};//BKPlanner
	
	enum plannerState
	{
		GOOD                 = 0,
		NEED_PARTIAL_REPLAN  = 1,
		NEED_FULL_REPLAN     = 2,
		NEED_RECOVERY        = 3,
		IN_RECOVERY          = 4
	};
	
	// Class encapsulates a thread that does planning work
	class BKPlanningThread
	{
	public:
		BKPlanningThread(BKPlanner* parent);
		void setFeeder(boost::weak_ptr<BKFeederThread> feeder);
		
		void run();
		
		// Escalates to a higher need of replanning/recovery
		void escalatePlannerState(plannerState newstate);
		void setPlannerState(plannerState newstate);
		plannerState getPlannerState();
		
		// Retrieve the path segments that the planning thread has committed
		bool segmentsAvailable();
		p_nav::Path dequeueSegments();
		// You can grab the mutex if you need to make multiple calls to these functions
		recursive_mutex          committed_path_mutex_;
	
		// If true, the planner ignores candidate goal generation and just returns the true goal.  Useful for directly testing point-point planning
		bool override_goal_generation_;
		
	private:
		// Non-owning references to the other threads
		// Whenever we want to dereference these we need to take ownership
		// ex: BKPlannerPtr(parent_weak_)->someFunction();
		BKPlanner* parent_;
		boost::weak_ptr<BKFeederThread> feeder_;

		// The distance of the commited path that the planner will try to maintain at all times (meters)
		double commit_distance_;
		// If planner can not get exactly to the target, it will try to get this far away
		double standoff_distance_;
		
		// The planner periodically checks if it is faster just to scrap the current path and replan
		ros::Time last_scrapped_path_;
		ros::Duration scrap_path_timeout_;
		
		// State variables for recovery
		ros::Time time_started_recovery_;
		ros::Time next_recovery_action_;
		int recovery_counter_;
		
		double short_dist_;
		double loop_rate_;
		
		// Manage planning
    void doRecovery();
		void startRecovery();
    bool doFullReplan();
    bool doPartialReplan();
    void commitPathSegments();
		void enqueueSegments(p_nav::Path new_segments);
		p_nav::PathSegment commitOneSegment();
		bool isTargetBehind(const PoseStamped& target);
		
		
		shared_ptr<bk_sbpl::BKSBPLLatticePlanner>  lattice_planner_;
		shared_ptr<segment_lib::SegmentVisualizer> visualizer_;
		
		
		p_nav::Path planner_path_;
		PoseStamped last_committed_pose_;
		int         last_committed_segnum_;

		// Save a copy of the goals for visualization
		ros::Publisher candidate_goal_pub_;
		ros::Publisher goal_pub_;
		geometry_msgs::PoseArray pub_goals_;
		
    // Do not directly access these variables, not thread-safe.
		plannerState                    planner_state_;
		recursive_mutex          planner_state_mutex_;
		p_nav::Path committed_path_;
		
		
		
		/* Planning functions in planning_lib.cpp */
		// Makes a plan from start to goal
		bool planPointToPoint(const PoseStamped& start,
					                const PoseStamped& goal,
					                p_nav::Path&  segment_plan,
		                      shared_ptr<path_checker::PathChecker>     path_checker,
                          shared_ptr<bk_sbpl::BKSBPLLatticePlanner> lattice_planner );
		
		// Makes a plan from start to as near as possible to goal
		bool planApproximateToGoal(const PoseStamped& start,
		                           const PoseStamped& goal,
		                           p_nav::Path&       path,
		                           shared_ptr<path_checker::PathChecker>     path_checker,
                               shared_ptr<bk_sbpl::BKSBPLLatticePlanner> lattice_planner );
    
    bool planShortDistance(const PoseStamped& start,
                           const PoseStamped& goal,
                           p_nav::Path&       path,
                           shared_ptr<path_checker::PathChecker> path_checker);
    
		PoseStamped getPoseOffset(const PoseStamped& pose, double dtheta, double dx);

		// Generates candidate goals centered on the true goal
		vector<PoseStamped>
		generatePotentialGoals(const PoseStamped& start,
		                       const PoseStamped& true_goal,
                           shared_ptr<path_checker::PathChecker> path_checker);
                                        
		vector<PoseStamped>
		generateNearGoals(const PoseStamped& start,
		                  const PoseStamped& true_goal,
                      shared_ptr<path_checker::PathChecker> path_checker);
                      
		vector<PoseStamped>
		generateFarGoals(const PoseStamped& start,
		                 const PoseStamped& true_goal,
                     shared_ptr<path_checker::PathChecker> path_checker);
                                  
	};//BKPlanningThread
	
	
	// Class encapsulates a thread that takes path segments from the planner and passes them to steering
	class BKFeederThread
	{
	public:	
		BKFeederThread(BKPlanner* parent);
		void setPlanner(boost::weak_ptr<BKPlanningThread> planner);
		void run();
	
		// Thread-safe functions for managing the state of the feeder
		void   setFeederEnabled(bool state);
		void   sendResetSignals();
		double getFeederDistLeft();
		
		// A lock on this mutex will prevent the feeder from running its main loop and cause a reset/halt
		recursive_mutex  feeder_lock_mutex_;
		
	private:
		// Non-owning references to the other threads
		// Whenever we want to dereference these we need to take ownership
		// ex: BKPlannerPtr(parent_weak_)->someFunction();
		BKPlanner* parent_;
		actionlib::SimpleActionClient<p_nav::ExecutePathAction> client_;
		boost::weak_ptr<BKPlanningThread> planner_;
		
		
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
		
		// Callbacks for action server
		void doneCb(const actionlib::SimpleClientGoalState& state,
                const p_nav::ExecutePathResultConstPtr& result);
		void activeCb();
		void feedbackCb(const p_nav::ExecutePathFeedbackConstPtr& feedback);
		
		// Keep around this many already-completed segments for show
		int    segs_to_trail_;
		double loop_rate_;
		
		p_nav::ExecutePathFeedback latest_feedback_; // Feedback from action server
		boost::mutex feedback_mutex_;
		p_nav::Path  feeder_path_;
		bool         feeder_path_has_changed_; // Has our path been updated?
		
		shared_ptr<segment_lib::SegmentVisualizer> visualizer_;
		bool client_has_goal_;
		
		// Do not directly access these variables, not thread-safe.
		bool feeder_enabled_;
		recursive_mutex feeder_enabled_mutex_, feeder_path_mutex_;
	};//BKFeederThread
	
	
	
};//namespace
#endif
