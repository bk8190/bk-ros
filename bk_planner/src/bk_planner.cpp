#include <bk_planner/bk_planner.h>

namespace bk_planner
{
BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	planner_costmap_    (NULL),
	lattice_planner_    (NULL),
	nh_                 (),
	priv_nh_            ("~"),
	tf_                 (tf),
	client_             ("execute_path", true)
	//planner_costmap_ros_("local_costmap", tf),
	//lattice_planner_    ("lattice_planner", &planner_costmap_ros_)
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
	
	planner_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
	lattice_planner_ = new bk_sbpl_lattice_planner::BKSBPLLatticePlanner("lattice_planner", planner_costmap_);
	path_checker_    = new path_checker::PathChecker("path_checker", planner_costmap_);
	segment_visualizer_ = new segment_lib::SegmentVisualizer(std::string("segment_visualization"));
	
//	dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
//	dynamic_reconfigure::Server<bk_planner::BKPlannerConfig>::CallbackType cb = boost::bind(&BKPlannerConfig::reconfigureCB, this, _1, _2);
//	dsrv_->setCallback(cb);	

	ROS_INFO("Waiting for server...");
	client_.waitForServer();
	
	ROS_INFO("BKPlanner constructor finished");
}

BKPlanner::~BKPlanner()
{
	//delete dsrv_;
	delete lattice_planner_;
	delete planner_costmap_;
	delete path_checker_;
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
  
void BKPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal_ptr)
{
	geometry_msgs::PoseStamped goal = *goal_ptr;

	ROS_INFO("Got new goal: (%.2f,%.2f) in frame %s", goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
	
	geometry_msgs::PoseStamped goal_transformed = poseToGlobalFrame(goal);
	
	this->makePlan(goal_transformed);
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
	ROS_INFO("Plan succeeded in %.2f seconds. Plan has %d segments.", t.toSec(), segment_plan.segs.size());
	
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

};// namespace


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bk_planner_node");
  tf::TransformListener tf(ros::Duration(10));

  bk_planner::BKPlanner bkp("bk_planner", tf);

  ros::spin();

  return(0);
}
