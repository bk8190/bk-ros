#include <bk_planner/bk_planner.h>


namespace bk_planner
{
BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	nh_                 (),
	tf_                 (tf),
	planner_costmap_ros_(NULL),
	lattice_planner_(NULL)
	//planner_costmap_ros_("local_costmap", tf),
	//lattice_planner_    ("lattice_planner", &planner_costmap_ros_)
{
	nh_.param("test_param", test_param_, 0.0);
	
	goal_sub_     = nh_.subscribe("goal", 1, &BKPlanner::goalCB, this);
	plan_pub_     = nh_.advertise<precision_navigation_msgs::Path>("plan", 1);
	plan_vis_pub_ = nh_.advertise<nav_msgs::Path>("plan_visualization", 1);

//	dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
//	dynamic_reconfigure::Server<bk_planner::BKPlannerConfig>::CallbackType cb = boost::bind(&BKPlannerConfig::reconfigureCB, this, _1, _2);
//	dsrv_->setCallback(cb);	
	ROS_INFO("BKPlanner constructor finished");
}

BKPlanner::~BKPlanner()
{
	//delete dsrv_;
}

void BKPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	ROS_INFO("Got new goal: (%.2f,%.2f)", goal->pose.position.x, goal->pose.position.y);
}

bool BKPlanner::makePlan(const geometry_msgs::PoseStamped& goal)
{
	

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
