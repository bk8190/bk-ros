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

	// Shorthand
	namespace p_nav = precision_navigation_msgs;
	
	const double pi  = 3.1415926;
	const double eps = .0001;    // Precision for floating point comparison

	double rect_angle(double t);

	void printPathSegment(const p_nav::PathSegment& s);
	
	// Reindexes the segment numbers, starting at start_seg_num.
	void reindexPath(p_nav::Path& path, int start_seg_num);
	
	// Returns the index of seg_num within path.  -1 if DNE.
	int segnumToIndex(const std::vector<p_nav::PathSegment>& segs, unsigned int seg_num);
	int segnumToIndex(const p_nav::Path& path, unsigned int seg_num);
	
	int getFirstSegnum(const p_nav::Path& path);
	int getLastSegnum(const p_nav::Path& path);
	
	// Gets the frame of the last pose, sets it as the frame for every segment and the path as a whole.
	void reFrame(p_nav::Path& path);
	
	double linDist(const p_nav::PathSegment& seg);
/* Path segment creation (create_seg.cpp) */
/*==============================================================================*/
	p_nav::PathSegment
	makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2);
	
	p_nav::PathSegment makeUndirectedLineSegment(double x1, double y1, double x2, double y2);
	// Makes a line segment respecting an initial angle (can handle backing up that way)
	p_nav::PathSegment makeDirectedLineSegment(double x1, double y1, double x2, double y2, double t1);	
	
	p_nav::PathSegment makeTurnSegment(double x, double y, double t1, double t2);
	p_nav::PathSegment makeArcSegment(double x1, double y1, double x2, double y2, double t1, double t2);
	
	
/* Path segment smoothing (smooth_path.cpp) */
/*==============================================================================*/

	// Combines some segments (ex. if there is a turn followed by an arc, replaces it by a single arc)
	p_nav::Path combineSegments(const p_nav::Path& path);

	p_nav::Path replaceTurnArcs(const p_nav::Path& path);
	p_nav::Path replaceMultipleTurns(const p_nav::Path& path);

	// Resamples the path's poses
	p_nav::Path
	smoothPath(const p_nav::Path& path);
	
	// Resamples the path's poses
	p_nav::Path
	smoothPathMultiple(const p_nav::Path& path, int passes);
	
	
/* Path segment interpolation (interp_seg.cpp) */
/*==============================================================================*/
	
	// Returns a vector of poses sampled from a segment path (same as calling interpSegment multiple times and concatenating)
	nav_msgs::Path
	interpPath(const p_nav::Path& path, double dx, double dtheta);

	// Returns a vector of poses sampled from a segment: the beginning, the end, and regular samples along distance dx (line/arc) and dtheta (spin in place).
	std::vector<geometry_msgs::PoseStamped>
	interpSegment(const p_nav::PathSegment& seg, double dx, double dtheta);

	std::vector<geometry_msgs::PoseStamped>
	interpLineSegment(const p_nav::PathSegment& seg, double dx);

	std::vector<geometry_msgs::PoseStamped>
	interpArcSegment(const p_nav::PathSegment& seg, double dtheta);

	std::vector<geometry_msgs::PoseStamped>
	interpSpinSegment(const p_nav::PathSegment& seg, double dtheta);
	
	geometry_msgs::PoseStamped
	getEndPose(const p_nav::PathSegment& seg);
	
	geometry_msgs::PoseStamped
	getStartPose(const p_nav::PathSegment& seg);
	
	// Returns whether or not seg specifies backward motion
	bool isLineSegmentReversed(p_nav::PathSegment seg);
	
};//namespace
#endif
