/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Mike Phillips
*********************************************************************/

#include <bk_sbpl_lattice_planner/bk_sbpl_lattice_planner.h>
#include <pluginlib/class_list_macros.h>
#include <bk_sbpl_lattice_planner/SBPLLatticePlannerStats.h>
using namespace std;
using namespace ros;

PLUGINLIB_REGISTER_CLASS(BKSBPLLatticePlanner, bk_sbpl_lattice_planner::BKSBPLLatticePlanner, nav_core::BaseGlobalPlanner);

namespace bk_sbpl_lattice_planner{

class LatticeSCQ : public StateChangeQuery{
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT* env, std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const{
      if(predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const{
      if(succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
};

BKSBPLLatticePlanner::BKSBPLLatticePlanner()
  : initialized_(false){
}

// Don't know why, but without this function I get linker errors.
// Maybe because this class was initialized from a base class with virtual members?
void
initialize(std::string name, Costmap2DROS* costmap_ros) {
	while(ros::ok()){
		ROS_FATAL("FAILFAILFAILFAILFAIL");
	}
}
void
BKSBPLLatticePlanner::initialize(std::string name, Costmap2DROS* costmap_ros){
	while(ros::ok()){
		ROS_FATAL("FAILFAILFAILFAILFAIL");
	}
}
    
  
BKSBPLLatticePlanner::BKSBPLLatticePlanner(std::string name, boost::shared_ptr<Costmap2DROS> costmap_ros) 
  : initialized_(false){
  initialize(name, costmap_ros);
}

void
BKSBPLLatticePlanner::initialize(std::string name, boost::shared_ptr<Costmap2DROS> costmap_ros){
  if(!initialized_){
    ros::NodeHandle private_nh("~/"+name);
    ros::NodeHandle nh(name);
    
    //ROS_INFO("Name is %s", name.c_str());
    private_nh.param("smooth_path"        , smooth_path_        , false);
    private_nh.param("planner_type"       , planner_type_       , string("ARAPlanner"));
    private_nh.param("allocated_time"     , allocated_time_     , 10.0);
    private_nh.param("initial_epsilon"    , initial_epsilon_    , 3.0);
    private_nh.param("environment_type"   , environment_type_   , string("XYThetaLattice"));
    private_nh.param("forward_search"     , forward_search_     , bool(false));
    private_nh.param("primitive_filename" , primitive_filename_ , string(""));
    private_nh.param("force_scratch_limit", force_scratch_limit_, 500);

		ROS_INFO("[bk_sbpl_lattice_planner] Using primitive file \"%s\"", primitive_filename_.c_str());
		if( smooth_path_ ){ ROS_WARN("[bk_sbpl_lattice_planner] Smoothing paths"); }
		
    double nominalvel_mpersecs, timetoturn45degsinplace_secs;
    private_nh.param("nominalvel_mpersecs"         , nominalvel_mpersecs, 0.4);
    private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

    int lethal_obstacle;
    private_nh.param("lethal_obstacle",lethal_obstacle,20);
    lethal_obstacle_ = (unsigned char) lethal_obstacle;
    inscribed_inflated_obstacle_ = lethal_obstacle_-1;
    sbpl_cost_multiplier_ = (unsigned char) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
    ROS_DEBUG("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz",lethal_obstacle,inscribed_inflated_obstacle_,sbpl_cost_multiplier_);
    
    costmap_ros_ = costmap_ros;
    costmap_ros_->clearRobotFootprint();
    costmap_ros_->getCostmapCopy(cost_map_);

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    if ("XYThetaLattice" == environment_type_){
      ROS_DEBUG("Using a 3D costmap for theta lattice\n");
      env_ = new EnvironmentNAVXYTHETALAT();
    }
    else{
      ROS_ERROR("XYThetaLattice is currently the only supported environment!\n");
      exit(1);
    }

    if(!env_->SetEnvParameter("cost_inscribed_thresh",costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))){
      ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
      exit(1);
    }
    if(!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", costMapCostToSBPLCost(cost_map_.getCircumscribedCost()))){
      ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
      exit(1);
    }
    int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV.push_back(pt);
    }

    bool ret;
    try{
      ret = env_->InitializeEnv(costmap_ros_->getSizeInCellsX(), // width
                                costmap_ros_->getSizeInCellsY(), // height
                                0, // mapdata
                                0, 0, 0, // start (x, y, theta, t)
                                0, 0, 0, // goal (x, y, theta)
                                0, 0, 0, //goal tolerance
                                perimeterptsV, costmap_ros_->getResolution(), nominalvel_mpersecs,
                                timetoturn45degsinplace_secs, obst_cost_thresh,
                                primitive_filename_.c_str());
    }
    catch(SBPL_Exception e){
      ROS_ERROR("SBPL encountered a fatal exception!");
      ret = false;
    }
    
    if(!ret){
      ROS_ERROR("SBPL initialization failed!");
      exit(1);
    }
    for (ssize_t ix(0); ix < costmap_ros_->getSizeInCellsX(); ++ix)
      for (ssize_t iy(0); iy < costmap_ros_->getSizeInCellsY(); ++iy)
        env_->UpdateCost(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));

    if ("ARAPlanner" == planner_type_){
      ROS_INFO("Planning with ARA*");
      planner_ = new ARAPlanner(env_, forward_search_);
    }
    else if ("ADPlanner" == planner_type_){
      ROS_INFO("Planning with AD*");
      planner_ = new ADPlanner(env_, forward_search_);
    }
    else{
      ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
      exit(1);
    }

    ROS_INFO("[sbpl_lattice_planner] Initialized successfully");
    //plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan_visualization", 1);
    stats_publisher_ = private_nh.advertise<bk_sbpl::SBPLLatticePlannerStats>("sbpl_lattice_planner_stats", 1);
    
    initialized_ = true;
  }
}
  
//Taken from Sachin's sbpl_cart_planner
//This rescales the costmap according to a rosparam which sets the obstacle cost
unsigned char
BKSBPLLatticePlanner::costMapCostToSBPLCost(unsigned char newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE)
    return lethal_obstacle_;
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;
  else
    return (unsigned char) (newcost/sbpl_cost_multiplier_ + 0.5);
}

void
BKSBPLLatticePlanner::publishStats(int solution_cost, int solution_size, 
                                   const PoseStamped& start, const PoseStamped& goal){
  // Fill up statistics and publish
  bk_sbpl::SBPLLatticePlannerStats stats;
  stats.initial_epsilon = initial_epsilon_;
  stats.plan_to_first_solution = false;
  stats.final_number_of_expands = planner_->get_n_expands();
  stats.allocated_time = allocated_time_;

  stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
  stats.actual_time = planner_->get_final_eps_planning_time();
  stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
  stats.final_epsilon = planner_->get_final_epsilon();

  stats.solution_cost = solution_cost;
  stats.path_size = solution_size;
  stats.start = start;
  stats.goal = goal;
  stats_publisher_.publish(stats);
}

bool
BKSBPLLatticePlanner::makePlan(const PoseStamped&        start,
                               const PoseStamped&        goal,
                               std::vector<PoseStamped>& plan)
{
	while(true){
		ROS_FATAL("Herp de derp this function doesn't actually exist");
	}
	return false;
}
                                 
bool
BKSBPLLatticePlanner::makeSegmentPlan(const PoseStamped& start,
                                      const PoseStamped& goal,
                                      p_nav::Path&                      segmentPlan)
{
  if(!initialized_){
    ROS_ERROR("Global planner is not initialized");
    return false;
  }

  segmentPlan.segs.clear();

  costmap_ros_->clearRobotFootprint();
  ROS_DEBUG("[sbpl_lattice_planner] robot footprint cleared");

  ROS_DEBUG("[sbpl_lattice_planner] getting fresh copy of costmap");
  costmap_ros_->getCostmapCopy(cost_map_);

  ROS_DEBUG("[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal  = 2 * atan2(goal .pose.orientation.z, goal .pose.orientation.w);

	// Set the start state in the planner
  try{
    int ret = env_->SetStart(start.pose.position.x - cost_map_.getOriginX(), start.pose.position.y - cost_map_.getOriginY(), theta_start);
    if(ret < 0 || planner_->set_start(ret) == 0){
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
    return false;
  }

	// Set the goal state in the planner
  try{
    int ret = env_->SetGoal(goal.pose.position.x - cost_map_.getOriginX(), goal.pose.position.y - cost_map_.getOriginY(), theta_goal);
    if(ret < 0 || planner_->set_goal(ret) == 0){
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
    return false;
  }
  
  
  // Get a list of all of the cell costs changed since the last planning iteration
  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  vector<nav2dcell_t> changedcellsV;

  for(unsigned int ix = 0; ix < cost_map_.getSizeInCellsX(); ix++) {
    for(unsigned int iy = 0; iy < cost_map_.getSizeInCellsY(); iy++) {

      unsigned char oldCost = env_->GetMapCost(ix,iy);
      unsigned char newCost = costMapCostToSBPLCost(cost_map_.getCost(ix,iy));

      if(oldCost == newCost) continue;

      allCount++;

      //first case - off cell goes on
      if((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        offOnCount++;
      }

			//second case - on cell goes off
      if((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        onOffCount++;
      }
      env_->UpdateCost(ix, iy, costMapCostToSBPLCost(cost_map_.getCost(ix,iy)));

      nav2dcell_t nav2dcell;
      nav2dcell.x = ix;
      nav2dcell.y = iy;
      changedcellsV.push_back(nav2dcell);
    }
  }

	// Update the planner with the new cell costs
  try{
    if(!changedcellsV.empty()){
      StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
      planner_->costs_changed(*scq);
      delete scq;
    }

    if(allCount > force_scratch_limit_)
      planner_->force_planning_from_scratch();
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL failed to update the costmap");
    return false;
  }

  //setting planner parameters
  ROS_DEBUG("allocated time:%f, init eps:%f\n",allocated_time_,initial_epsilon_);
  planner_->set_initialsolution_eps(initial_epsilon_);
  planner_->set_search_mode(false);

	// Run the planner
  ROS_DEBUG("[sbpl_lattice_planner] run planner");
  vector<int> solution_stateIDs;
  int solution_cost;
  try{
    int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
    if(ret)
      ROS_DEBUG("Solution is found\n");
    else{
      ROS_DEBUG("Solution not found\n");
      publishStats(solution_cost, 0, start, goal);
      return false;
    }
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());
  ros::Time plan_time = ros::Time::now();

	// The planner returned a state ID path.  Convert this into a path-segment path.
  vector<EnvNAVXYTHETALAT3Dpt_t>  sbpl_path;   // Plan of points for visualization
  try{
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
    
    const double pi = 3.1415926;
    //ROS_INFO("[sbpl] Angles corrected from (%.3fpi->%.3fpi) to (%.3fpi->%.3fpi)", sbpl_path.front().theta/pi, sbpl_path.back().theta/pi, tf::getYaw(start.pose.orientation)/pi, tf::getYaw( goal.pose.orientation)/pi);
    
    // Correct discretization error: make the first and last points in the path have the same angles as the start/goal points
    if( sbpl_path.size() > 0 )
    {
		  sbpl_path.front().theta = tf::getYaw(start.pose.orientation);
		  sbpl_path.front().x     = start.pose.position.x;
		  sbpl_path.front().y     = start.pose.position.y;
		  sbpl_path.back ().theta = tf::getYaw(goal.pose.orientation);
		  sbpl_path.back ().x     = goal.pose.position.x;
		  sbpl_path.back ().y     = goal.pose.position.y;
    }
    
    // Added: convert the state ID path into path segments
    segmentPlan.header.frame_id = costmap_ros_->getGlobalFrameID();
    segmentPlan.header.stamp    = plan_time;
    double dx = cost_map_.getOriginX();
    double dy = cost_map_.getOriginY();
    ConvertStateIDPathintoSegmentPath(env_, solution_stateIDs, segmentPlan, dx, dy);
  }
  catch(SBPL_Exception e){
    ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  
  // Combine some segments together (ex. merge consecutive turn-in-place segments)
  segmentPlan = segment_lib::combineSegments(segmentPlan);
  
  // Smooth out the plan to eliminate angular discontinuities
  if( smooth_path_ )
  	segmentPlan = segment_lib::smoothPath(segmentPlan);
	
  ROS_DEBUG("Plan has %d path segments.\n", (int)segmentPlan.segs.size());
  publishStats(solution_cost, sbpl_path.size(), start, goal);

  return true;
}


void
BKSBPLLatticePlanner::ConvertStateIDPathintoSegmentPath(EnvironmentNAVXYTHETALAT* env, const vector<int>& stateIDPath, p_nav::Path& segmentPath, double dx, double dy)
{
	// Discrete state
	int x1_c, y1_c, t1_c;
	int x2_c, y2_c, t2_c;
	
	// Continuous state
	double x1, y1, t1;
	double x2, y2, t2;

	p_nav::PathSegment this_seg;
	bool ret1, ret2;
	int sourceID, targetID;
	segmentPath.segs.clear();
	
	// Make a path segment for every change in state
	for(int path_index = 0; path_index < (int)(stateIDPath.size())-1; path_index++)
	{
		// IDs of the source/target state
		sourceID = stateIDPath.at(path_index  );
		targetID = stateIDPath.at(path_index+1);

		// Retrieve state information about the source and target
		env->GetCoordFromState(sourceID, x1_c, y1_c, t1_c);
		env->GetCoordFromState(targetID, x2_c, y2_c, t2_c);
		
		ret1 = env->PoseDiscToCont(x1_c, y1_c, t1_c, x1, y1, t1);		
		ret2 = env->PoseDiscToCont(x2_c, y2_c, t2_c, x2, y2, t2);
		
		if(ret1<0 || ret2<0){
			ROS_ERROR("Error converting discrete pose to continuous");
			segmentPath.segs.clear();
			return;
		}
		
		// We now have source and destination states.  Build a path segment.
		//ROS_INFO("Segment  %d: (%.2f,%.2f)->(%.2f,%.2f), %.2fpi -> %.2fpi", path_index, x1+dx, y1+dy, x2+dx, y2+dy, t1/pi, t2/pi);
		
		this_seg = segment_lib::makePathSegment(x1+dx,y1+dy,t1, x2+dx,y2+dy,t2); 
		
		// Check the endpoints
		/*std::vector<PoseStamped> interp = segment_lib::interpSegment(this_seg, 1, 1);
		geometry_msgs::Pose start_pose = interp.front().pose;
		geometry_msgs::Pose end_pose   = interp.back().pose;*/
		
		//ROS_INFO("Actually, : (%.2f,%.2f)->(%.2f,%.2f), %.2fpi -> %.2fpi", start_pose.position.x, start_pose.position.y, end_pose.position.x, end_pose.position.y, segment_lib::rect_angle(tf::getYaw(start_pose.orientation))/pi, segment_lib::rect_angle(tf::getYaw(end_pose.orientation)/pi));
		
		// ROS_INFO(".");
		
		this_seg.header.frame_id   = segmentPath.header.frame_id;
		this_seg.header.stamp      = segmentPath.header.stamp;
		this_seg.header.seq        = segmentPath.header.seq;
		this_seg.seg_number        = path_index;
		segmentPath.segs.push_back(this_seg);
	}
}

};//namespace
