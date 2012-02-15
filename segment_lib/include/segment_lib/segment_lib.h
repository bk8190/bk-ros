#ifndef SEGMENT_LIB_H_
#define SEGMENT_LIB_H_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <precision_navigation_msgs/Path.h>
#include <precision_navigation_msgs/PathSegment.h>

namespace segment_lib {

double rect_angle(double t);

precision_navigation_msgs::PathSegment makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2);

void printPathSegment(const precision_navigation_msgs::PathSegment& s);

// Returns a vector of poses from a segment: the beginning, the end, and regular samples along distance dx (line/arc) and dtheta (spin in place).
std::vector<geometry_msgs::Pose> interpSegment(const precision_navigation_msgs::PathSegment& s, double dx, double dtheta);

class SegmentVisualization
{
	public:
		SegmentVisualization(std::string name);
		~SegmentVisualization();
	
		void publishVisualization(const precision_navigation_msgs::Path& path);
		
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		
		ros::Publisher  vis_pub_;
};

};//namespace
#endif
