#include <bk_planner/bk_planner.h>

namespace bk_planner {

BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	nh_           (),
	priv_nh_      ("~"),
	tf_           (tf),
	got_new_goal_ (false)
{
	// Initialize the cost map and path checker
	planner_costmap_ = shared_ptr<costmap_2d::Costmap2DROS>
		(new costmap_2d::Costmap2DROS("local_costmap", tf_) );	          
	path_checker_    = shared_ptr<path_checker::PathChecker>
		(new path_checker::PathChecker("path_checker", planner_costmap_));
	
	
	priv_nh_.param("goal_hysteresis", goal_hysteresis_, 0.2);
	ROS_INFO("[bk_planner] Goal hysteresis is %.2f", goal_hysteresis_);
	
	// Get the robot's current pose and set a goal there
	tf::Stamped<tf::Pose> robot_pose;
	planner_costmap_->getRobotPose(robot_pose);
	PoseStamped start_pose;
	tf::poseStampedTFToMsg(robot_pose, start_pose);
	start_pose = poseToGlobalFrame(start_pose);
	setNewGoal(start_pose);
	
	// Create the planner and feeder threads
	planner_ = shared_ptr<BKPlanningThread>
		(new BKPlanningThread(this) );		
	feeder_   = shared_ptr<BKFeederThread>
		(new BKFeederThread  (this) );
	
	// Set their references to each other
	planner_->setFeeder (feeder_);
	feeder_ ->setPlanner(planner_);
	
	// Kick off the threads
	planning_thread_ = shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKPlanningThread::run, planner_)) );
	feeder_thread_   = shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKFeederThread::run  , feeder_ )) );

	// This node subscribes to a goal pose.
	goal_sub_ = nh_.subscribe("goal", 1, &BKPlanner::goalCB, this);
	
	// ROS_INFO("BKPlanner constructor finished");
	return;
}

BKPlanner::~BKPlanner()
{
	terminateThreads();
}

void
BKPlanner::terminateThreads()
{
	planning_thread_->interrupt();
	feeder_thread_  ->interrupt();
	planning_thread_->join();
	feeder_thread_  ->join();
}

double dist( const PoseStamped& p1, const PoseStamped& p2 )
{
	return sqrt(
              ((p1.pose.position.x - p2.pose.position.x)
              *(p1.pose.position.x - p2.pose.position.x))+
              ((p1.pose.position.y - p2.pose.position.y)
              *(p1.pose.position.y - p2.pose.position.y))+
              ((p1.pose.position.z - p2.pose.position.z)
              *(p1.pose.position.z - p2.pose.position.z)) );
}

void
BKPlanner::goalCB(const PoseStamped::ConstPtr& goal_ptr)
{
	PoseStamped curr_goal = getLatestGoal();

	ROS_INFO("[Goal callback] Got new goal: (%.2f,%.2f) in frame %s", goal_ptr->pose.position.x, goal_ptr->pose.position.y, goal_ptr->header.frame_id.c_str());
	
	PoseStamped new_goal = poseToGlobalFrame(*goal_ptr);
	
	double d = dist(curr_goal, new_goal);
	if( d > goal_hysteresis_ )
	{
		setNewGoal(new_goal);
		got_new_goal_ = true;
	}
}

PoseStamped
BKPlanner::poseToGlobalFrame(const PoseStamped& pose_msg)
{
	std::string global_frame = planner_costmap_->getGlobalFrameID();
	tf::Stamped<tf::Pose> goal_pose, global_pose;
	poseStampedMsgToTF(pose_msg, goal_pose);

	//just get the latest available transform
	goal_pose.stamp_ = ros::Time();

	try {
		tf_.transformPose(global_frame, goal_pose, global_pose);
	}
	catch(tf::TransformException& ex) {
		ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
		goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return pose_msg;
	}

	PoseStamped global_pose_msg;
	tf::poseStampedTFToMsg(global_pose, global_pose_msg);
	return global_pose_msg;
}

void 
BKPlanner::setNewGoal(PoseStamped new_goal)
{
	boost::recursive_mutex::scoped_try_lock l(goal_mutex_);
	while(!l){
		l = boost::recursive_mutex::scoped_try_lock(goal_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	got_new_goal_ = true;
	latest_goal_  = new_goal;
}

bool 
BKPlanner::gotNewGoal()
{
	boost::recursive_mutex::scoped_try_lock l(goal_mutex_);
	while(!l){
		l = boost::recursive_mutex::scoped_try_lock(goal_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	return got_new_goal_;
}

PoseStamped 
BKPlanner::getLatestGoal()
{
	boost::recursive_mutex::scoped_try_lock l(goal_mutex_);
	while(!l){
		l = boost::recursive_mutex::scoped_try_lock(goal_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	got_new_goal_ = false;
	return latest_goal_;
}

};// namespace

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "bk_planner_node");
	ros::NodeHandle nh;
	ros::Duration(3.0).sleep();
	tf::TransformListener tf(ros::Duration(10));
	
	/*bool found = false;
	while( !found ){
		try{
			ROS_INFO("[planner] Getting transforms");
			found = true;
			tf::StampedTransform transform;
			tf.waitForTransform("odom", "map"      , ros::Time::now(), ros::Duration(1));
			tf.lookupTransform ("odom", "map"      , ros::Time::now(), transform);
			tf.waitForTransform("odom", "base_link", ros::Time::now(), ros::Duration(1));
			tf.lookupTransform ("odom", "base_link", ros::Time::now(), transform);
		}
		catch(tf::TransformException& ex){
			ROS_INFO("[planner] Failed to get transforms %s", ex.what());
			found = false;
		}
	}
	ROS_INFO("[planner] Got transforms");*/

	try	{
		bk_planner::BKPlanner bkp("bk_planner", tf);

		// All callbacks are taken care of in the main thread
		while(bkp.nh_.ok()) {
			ros::spinOnce();
		}
	}
	// The planner will go out of scope here and call its destructor.
	// It might throw a lock error because of interraction between the threads and mutexes
	catch(boost::lock_error e) {
		cout << "Boost threw a lock error (" << e.what() << ")\n";
		cout << "Honey Badger don't care, Honey Badger don't give a &$%#\n\n";
	}
	
	return(0);
}
