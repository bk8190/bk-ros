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

	const double pi  = 3.1415926;
	const double eps = .0001;    // Precision for floating point comparison

	double rect_angle(double t);

	void printPathSegment(const precision_navigation_msgs::PathSegment& s);
	
	// Reindexes the segment numbers, starting at start_seg_num.
	void reindexPath(precision_navigation_msgs::Path& path, int start_seg_num);
	
	// Returns the index of seg_num within path.  -1 if DNE.
	int segnumToIndex(const std::vector<precision_navigation_msgs::PathSegment>& segs, unsigned int seg_num);
	int segnumToIndex(const precision_navigation_msgs::Path& path, unsigned int seg_num);
	
	int getFirstSegnum(const precision_navigation_msgs::Path& path);
	int getLastSegnum(const precision_navigation_msgs::Path& path);
	
/* Path segment creation (create_seg.cpp) */
/*==============================================================================*/
	precision_navigation_msgs::PathSegment
	makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2);
	
	precision_navigation_msgs::PathSegment makeUndirectedLineSegment(double x1, double y1, double x2, double y2);
	// Makes a line segment respecting an initial angle (can handle backing up that way)
	precision_navigation_msgs::PathSegment makeDirectedLineSegment(double x1, double y1, double x2, double y2, double t1);	
	
	precision_navigation_msgs::PathSegment makeTurnSegment(double x, double y, double t1, double t2);
	precision_navigation_msgs::PathSegment makeArcSegment(double x1, double y1, double x2, double y2, double t1, double t2);
	
	
/* Path segment smoothing (smooth_path.cpp) */
/*==============================================================================*/

	// Combines some segments (ex. if there is a turn followed by an arc, replaces it by a single arc)
	precision_navigation_msgs::Path combineSegments(const precision_navigation_msgs::Path& path);

	precision_navigation_msgs::Path replaceTurnArcs(const precision_navigation_msgs::Path& path);
	precision_navigation_msgs::Path replaceMultipleTurns(const precision_navigation_msgs::Path& path);

	// Resamples the path's poses
	precision_navigation_msgs::Path
	smoothPath(const precision_navigation_msgs::Path& path);
	
	// Resamples the path's poses
	precision_navigation_msgs::Path
	smoothPathMultiple(const precision_navigation_msgs::Path& path, int passes);
	
	
/* Path segment interpolation (interp_seg.cpp) */
/*==============================================================================*/
	
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
	
	geometry_msgs::PoseStamped
	getEndPose(const precision_navigation_msgs::PathSegment& seg);
	
	geometry_msgs::PoseStamped
	getStartPose(const precision_navigation_msgs::PathSegment& seg);
	
	// Returns whether or not seg specifies backward motion
	bool isLineSegmentReversed(precision_navigation_msgs::PathSegment seg);
	
};//namespace
#endif
