#include <bk_planner/bk_planner.h>

namespace bk_planner
{
BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	nh_      (),
	priv_nh_ ("~"),
	tf_      (tf),
	client_  ("execute_path", true)
	
{
	priv_nh_.param("speeds/max_vel_x"         ,max_vel_x_         ,0.0);
	priv_nh_.param("speeds/max_rotational_vel",max_rotational_vel_,0.0);
	priv_nh_.param("speeds/acc_lim_th"        ,acc_lim_th_        ,0.0);
	priv_nh_.param("speeds/acc_lim_x"         ,acc_lim_x_         ,0.0);
	priv_nh_.param("speeds/acc_lim_y"         ,acc_lim_y_         ,0.0);
	priv_nh_.param("planning/commit_distance" ,commit_distance_   ,1.1);
	priv_nh_.param("planning/segs_to_trail"   ,segs_to_trail_     ,4);
	
	ROS_INFO("Max speeds: (x,theta)   = (%.2f,%.2f)"     , max_vel_x_, max_rotational_vel_);
	ROS_INFO("Max accels: (x,theta,y) = (%.2f,%.2f,%.2f)", acc_lim_x_, acc_lim_th_, acc_lim_y_);
	ROS_INFO("Commit distance %.2f", commit_distance_);

	// This node subscribes to a goal pose.
	goal_sub_     = nh_.subscribe("goal", 1, &BKPlanner::goalCB, this);
	
	planner_costmap_ = boost::shared_ptr<costmap_2d::Costmap2DROS>
		(new costmap_2d::Costmap2DROS("local_costmap", tf_) );
	                 
	lattice_planner_ = boost::shared_ptr<bk_sbpl_lattice_planner::BKSBPLLatticePlanner>
		(new bk_sbpl_lattice_planner::BKSBPLLatticePlanner("lattice_planner", planner_costmap_) );
	                 
	path_checker_    = boost::shared_ptr<path_checker::PathChecker>
		(new path_checker::PathChecker("path_checker", planner_costmap_));
	
//	dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
//	dynamic_reconfigure::Server<bk_planner::BKPlannerConfig>::CallbackType cb = boost::bind(&BKPlannerConfig::reconfigureCB, this, _1, _2);
//	dsrv_->setCallback(cb);	

	// Get the robot's current pose and set a goal there
	tf::Stamped<tf::Pose> robot_pose;
	planner_costmap_->getRobotPose(robot_pose);
	geometry_msgs::PoseStamped start_pose;
	tf::poseStampedTFToMsg(robot_pose, start_pose);
	start_pose = poseToGlobalFrame(start_pose);
	setNewGoal(start_pose);

	ROS_INFO("Waiting for action server...");
	client_.waitForServer();
	
	// Initialize state variables
	feeder_path_.segs.clear();
	planner_path_.segs.clear();
	
	planner_state_         = GOOD;
	last_committed_segnum_ = 0;
	got_new_goal_          = false;
	feeder_enabled_        = false;
	client_has_goal_       = false;
	last_committed_pose_   = start_pose;
	latest_feedback_.current_segment.seg_number = 0;
	latest_feedback_.seg_distance_done = 0;0;
	
	// Kick off the threads
	planning_thread_    = boost::shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKPlanner::runPlanningThread, this)) );
	
	feeder_thread_ = boost::shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKPlanner::runFeederThread  , this)) );

	ROS_INFO("BKPlanner constructor finished");
	return;
}

BKPlanner::~BKPlanner()
{
	terminateThreads();
}

void BKPlanner::terminateThreads()
{
		// Terminate the threads
		planning_thread_->interrupt();
		planning_thread_->join();
	
		feeder_thread_->interrupt();
		feeder_thread_->join();

}

void BKPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal_ptr)
{
	geometry_msgs::PoseStamped goal = *goal_ptr;

	ROS_INFO("Got new goal: (%.2f,%.2f) in frame %s", goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
	
	setNewGoal(poseToGlobalFrame(goal));
	escalatePlannerState(NEED_PARTIAL_REPLAN);
}

geometry_msgs::PoseStamped BKPlanner::poseToGlobalFrame(const geometry_msgs::PoseStamped& pose_msg)
{
	std::string global_frame = planner_costmap_->getGlobalFrameID();
	tf::Stamped<tf::Pose> goal_pose, global_pose;
	poseStampedMsgToTF(pose_msg, goal_pose);
	
	//just get the latest available transform
	goal_pose.stamp_ = ros::Time();
	
	try{
		tf_.transformPose(global_frame, goal_pose, global_pose);
	}
	catch(tf::TransformException& ex){
		ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
		goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return pose_msg;
	}
	
	geometry_msgs::PoseStamped global_pose_msg;
	tf::poseStampedTFToMsg(global_pose, global_pose_msg);
	return global_pose_msg;
}


/*void BKPlanner::reconfigureCB(bk_planner::BKPlannerConfig &config, uint32_t level)
{
	boost::recursive_mutex::scoped_lock l(configuration_mutex_);
	ROS_INFO("Dynamic reconfigure callback");
}*/


      

};// namespace

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bk_planner_node");
	tf::TransformListener tf(ros::Duration(10));

	try	{
		bk_planner::BKPlanner bkp("bk_planner", tf);

		// All callbacks are taken care of in the main thread
		while(bkp.nh_.ok()) {
			ros::spinOnce();
		}
	}
	// The planner will go out of scope here and call its destructor.
	// It throws a lock error, possibly because terminate() is called on the threads.
	catch(boost::lock_error e) {
		cout << "Boost threw a lock error.  Honey badger don't care honey badger don't give a &$%#\n";
	}

	std::cout << "[bk_planner_node] Quitting" << std::endl;
	ros::shutdown();
  return(0);
}
