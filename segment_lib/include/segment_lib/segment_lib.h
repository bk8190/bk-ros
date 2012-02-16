#ifndef SEGMENT_LIB_H_
#define SEGMENT_LIB_H_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>

#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <precision_navigation_msgs/Path.h>
#include <precision_navigation_msgs/PathSegment.h>

namespace segment_lib {

	const double pi = 3.1415926;

	double rect_angle(double t);

	precision_navigation_msgs::PathSegment
	makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2);
	precision_navigation_msgs::PathSegment
	makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2, double& end_angle);
	
	void printPathSegment(const precision_navigation_msgs::PathSegment& s);

	// Resamples the path's poses
	precision_navigation_msgs::Path
	smoothPath(const precision_navigation_msgs::Path& path);
	
	// Resamples the path's poses
	precision_navigation_msgs::Path
	smoothPathMultiple(const precision_navigation_msgs::Path& path, int passes);
	
	// Returns a vector of poses sampled from a segment path (same as calling interpSegment multiple times and concatenating)
	nav_msgs::Path
	interpPath(const precision_navigation_msgs::Path& path, double dx, double dtheta);

	// Returns a vector of poses sampled from a segment: the beginning, the end, and regular samples along distance dx (line/arc) and dtheta (spin in place).
	std::vector<geometry_msgs::PoseStamped>
	interpSegment(const precision_navigation_msgs::PathSegment& seg, double dx, double dtheta);

	std::vector<geometry_msgs::PoseStamped>
	interpLineSegment(const precision_navigation_msgs::PathSegment& seg, double dx);

	std::vector<geometry_msgs::PoseStamped>
	interpArcSegment(const precision_navigation_msgs::PathSegment& seg, double dtheta);

	std::vector<geometry_msgs::PoseStamped>
	interpSpinSegment(const precision_navigation_msgs::PathSegment& seg, double dtheta);

};//namespace
#endif
