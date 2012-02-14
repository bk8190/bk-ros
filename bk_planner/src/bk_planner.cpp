#include <bk_planner/bk_planner.h>


namespace bk_planner
{
BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	nh_                 (),
	priv_nh_            ("~"),
	tf_                 (tf),
	planner_costmap_    (NULL),
	lattice_planner_    (NULL)
	//planner_costmap_ros_("local_costmap", tf),
	//lattice_planner_    ("lattice_planner", &planner_costmap_ros_)
{

/*
	// Test to see if parameters are working
	priv_nh_.param("test_param", test_param_, 0.0);
	double vel;
	priv_nh_.param("speeds/max_vel_x", vel, 0.0);
	ROS_INFO("Param vel  = %.2f", vel);
	ROS_INFO("Param test = %.2f", test_param_);
*/

	// This node subscribes to a goal pose.  It publishes a segment plan, and a standard ROS path for visualization
	goal_sub_     = nh_.subscribe("goal", 1, &BKPlanner::goalCB, this);
	plan_pub_     = nh_.advertise<precision_navigation_msgs::Path>(name + "/plan", 1);
	plan_vis_pub_ = nh_.advertise<nav_msgs::Path>(name + "/plan_visualization", 1);

	planner_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
	lattice_planner_ = new bk_sbpl_lattice_planner::BKSBPLLatticePlanner("lattice_planner", planner_costmap_);
	
//	dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
//	dynamic_reconfigure::Server<bk_planner::BKPlannerConfig>::CallbackType cb = boost::bind(&BKPlannerConfig::reconfigureCB, this, _1, _2);
//	dsrv_->setCallback(cb);	

	ROS_INFO("BKPlanner constructor finished");
}

BKPlanner::~BKPlanner()
{
	//delete dsrv_;
	delete lattice_planner_;
	delete planner_costmap_;
}


geometry_msgs::PoseStamped BKPlanner::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
{
	std::string global_frame = planner_costmap_->getGlobalFrameID();
	tf::Stamped<tf::Pose> goal_pose, global_pose;
	poseStampedMsgToTF(goal_pose_msg, goal_pose);
	
	//just get the latest available transform... for accuracy they should send
	//goals in the frame of the planner
	goal_pose.stamp_ = ros::Time();
	
	try{
		tf_.transformPose(global_frame, goal_pose, global_pose);
	}
	catch(tf::TransformException& ex){
		ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
		goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return goal_pose_msg;
	}
	
	geometry_msgs::PoseStamped global_pose_msg;
	tf::poseStampedTFToMsg(global_pose, global_pose_msg);
	return global_pose_msg;
}
  
void BKPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal_ptr)
{
	geometry_msgs::PoseStamped goal = *goal_ptr;

	ROS_INFO("Got new goal: (%.2f,%.2f) in frame %s", goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
	
	geometry_msgs::PoseStamped goal_transformed = goalToGlobalFrame(goal);
	
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

	nav_msgs::Path vis_plan;
	precision_navigation_msgs::Path segment_plan;
	
	if( !start.header.frame_id.compare(goal.header.frame_id) ){
		ROS_ERROR("Start and goal poses are in different frames.  What are you trying to pull here?");
    
	ROS_INFO("Planning.");
	bool ret = lattice_planner_->makeSegmentPlan(start, goal, vis_plan.poses, segment_plan);

	if( ret == false ){
		ROS_ERROR("Planner failed!");
		return false;
	}
	
	ROS_INFO("Plan succeeded. Visualization has %ld points",vis_plan.poses.size());
	
	vis_plan.header.stamp        = ros::Time::now();
	vis_plan.header.frame_id     = start.header.frame_id;
	segment_plan.header.stamp    = ros::Time::now();
	segment_plan.header.frame_id = start.header.frame_id;
	
	plan_vis_pub_.publish(vis_plan);
	plan_pub_    .publish(segment_plan);
	
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
