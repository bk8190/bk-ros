#include <bk_planner/bk_planner.h>

namespace bk_planner
{
BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	nh_                 (),
	priv_nh_            ("~"),
	tf_                 (tf),
	client_             ("execute_path", true),
	got_new_goal_       (false)
{

	priv_nh_.param("speeds/max_vel_x"         ,max_vel_x_         ,0.0);
	priv_nh_.param("speeds/max_rotational_vel",max_rotational_vel_,0.0);
	priv_nh_.param("speeds/acc_lim_th"        ,acc_lim_th_        ,0.0);
	priv_nh_.param("speeds/acc_lim_x"         ,acc_lim_x_         ,0.0);
	priv_nh_.param("speeds/acc_lim_y"         ,acc_lim_y_         ,0.0);
	
	ROS_INFO("Max speeds: (x,theta)   = (%.2f,%.2f)"     , max_vel_x_, max_rotational_vel_);
	ROS_INFO("Max accels: (x,theta,y) = (%.2f,%.2f,%.2f)", acc_lim_x_, acc_lim_th_, acc_lim_y_);
/*
	// Test to see if parameters are working
	priv_nh_.param("test_param", test_param_, 0.0);
	double vel;
	ROS_INFO("Param vel  = %.2f", vel);
	ROS_INFO("Param test = %.2f", test_param_);
*/

	// This node subscribes to a goal pose.  It publishes a segment plan, and a standard ROS path for visualization
	goal_sub_     = nh_.subscribe("goal", 1, &BKPlanner::goalCB, this);
	plan_pub_     = nh_.advertise<precision_navigation_msgs::Path>(name + "/plan", 1);
	
	planner_costmap_ = boost::shared_ptr<costmap_2d::Costmap2DROS>
		(new costmap_2d::Costmap2DROS("local_costmap", tf_) );
	                 
	lattice_planner_ = boost::shared_ptr<bk_sbpl_lattice_planner::BKSBPLLatticePlanner>
		(new bk_sbpl_lattice_planner::BKSBPLLatticePlanner("lattice_planner", planner_costmap_) );
	                 
	path_checker_    = boost::shared_ptr<path_checker::PathChecker>
		(new path_checker::PathChecker("path_checker", planner_costmap_));
	                 
	segment_visualizer_ = boost::shared_ptr<segment_lib::SegmentVisualizer>
		(new segment_lib::SegmentVisualizer(std::string("segment_visualization")) );
	
//	dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
//	dynamic_reconfigure::Server<bk_planner::BKPlannerConfig>::CallbackType cb = boost::bind(&BKPlannerConfig::reconfigureCB, this, _1, _2);
//	dsrv_->setCallback(cb);	

	// Set up the threads
	planning_thread_    = boost::shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKPlanner::planningThread,   this)) );
	
	path_feeder_thread_ = boost::shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKPlanner::pathFeederThread, this)) );
		
	// Kick off the threads

	ROS_INFO("Waiting for server...");
	client_.waitForServer();
	
	ROS_INFO("BKPlanner constructor finished");
}

BKPlanner::~BKPlanner()
{
	// Terminate the threads
	planning_thread_->interrupt();
	planning_thread_->join();
	
	path_feeder_thread_->interrupt();
	path_feeder_thread_->join();
}


void BKPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal_ptr)
{
	geometry_msgs::PoseStamped goal = *goal_ptr;

	ROS_INFO("Got new goal: (%.2f,%.2f) in frame %s", goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
	
	latest_goal_  = poseToGlobalFrame(goal);
	got_new_goal_ = true;
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
  
  
bool BKPlanner::makePlan(const geometry_msgs::PoseStamped& goal)
{
	// Get the robot's current pose
	tf::Stamped<tf::Pose> robot_pose;
	if(!planner_costmap_->getRobotPose(robot_pose)){
		ROS_ERROR("bk_planner cannot make a plan for you because it could not get the start pose of the robot");
		return false;
	}
	geometry_msgs::PoseStamped start;
	tf::poseStampedTFToMsg(robot_pose, start);

	precision_navigation_msgs::Path segment_plan;
	
	/*if( !start.header.frame_id.compare(goal.header.frame_id) ){
		ROS_ERROR("Start and goal poses are in different frames.  What are you trying to pull here?");
		return false;
	}*/
    
  ros::Time t1 = ros::Time::now();
	bool ret = lattice_planner_->makeSegmentPlan(start, goal, segment_plan);

	if( ret == false ){
		ROS_ERROR("Planner failed!");
		return false;
	}
	ros::Duration t = ros::Time::now() - t1;
	
	ROS_INFO("Plan succeeded in %.2f seconds. Plan has %lu segments.", t.toSec(), segment_plan.segs.size());
	
	// Get safe velocities for the segments
	path_checker_->assignPathVelocity(segment_plan);
	segment_plan.header.stamp    = ros::Time::now();
	segment_plan.header.frame_id = start.header.frame_id;
	
	// Publish the plan, and have the visualizer publish visualization
	plan_pub_.publish(segment_plan);
	segment_visualizer_->publishVisualization(segment_plan);
	
	// Temporary: execute the whole plan
	client_.waitForServer();
	precision_navigation_msgs::ExecutePathGoal action_goal;
	action_goal.segments = segment_plan.segs;
	client_.sendGoal(action_goal);
	
	return true;
}



/*void BKPlanner::reconfigureCB(bk_planner::BKPlannerConfig &config, uint32_t level)
{
	boost::recursive_mutex::scoped_lock l(configuration_mutex_);
	ROS_INFO("Dynamic reconfigure callback");
}*/

void BKPlanner::planningThread()
{
	ROS_INFO("bk_planner planning thread started");
	ros::Rate r = 1.0;// planner_frequency_;
	ros::NodeHandle n;
	
	while(n.ok())
	{
		while( !got_new_goal_ )
			r.sleep();
		got_new_goal_ = false;
		
		ROS_INFO("Planning thread got a new goal");
		this->makePlan(latest_goal_);

		r.sleep();
	}
	
	ROS_INFO("Planning thread done.");
}
      

void BKPlanner::pathFeederThread()
{
	ROS_INFO("bk_planner path feeder thread started");
	ros::Rate r = 1.0; //path_feeder_frequency_;
	ros::NodeHandle n;
	
	while(n.ok())
	{
	
		r.sleep();
	}
	
	ROS_INFO("Path feeder thread done.");
}
      

};// namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bk_planner_node");
  tf::TransformListener tf(ros::Duration(10));

  bk_planner::BKPlanner bkp("bk_planner", tf);

	// All callbacks are taken care of in the main thread
	while(ros::ok())
	{
  	ros::spinOnce();
	}

	
	
	ROS_INFO("bk_planner finished.");
  return(0);
}
