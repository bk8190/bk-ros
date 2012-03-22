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
	
	using     geometry_msgs::PoseStamped;
	using     p_nav        ::PathSegment;
	using     geometry_msgs::Point;
	using     std          ::vector;
	
	const double pi  = 3.1415926;
	const double eps = .0001;    // Precision for floating point comparison

	double rect_angle(double t);

	void printPathSegment(const PathSegment& s);
	
	// Reindexes the segment numbers, starting at start_seg_num.
	void reindexPath(p_nav::Path& path, int start_seg_num);
	
	// Returns the index of seg_num within path.  -1 if DNE.
	int segnumToIndex(const vector<PathSegment>& segs, unsigned int seg_num);
	int segnumToIndex(const p_nav::Path& path, unsigned int seg_num);
	
	int getFirstSegnum(const p_nav::Path& path);
	int getLastSegnum(const p_nav::Path& path);
	
	// Gets the frame of the last pose, sets it as the frame for every segment and the path as a whole.
	void reFrame(p_nav::Path& path);
	
	double linDist(const PathSegment& seg);
	
	//  public domain function by Darel Rex Finley, 2006
	//  Determines the intersection point of the line segment defined by points A and B
	//  with the line segment defined by points C and D.
	bool lineSegmentIntersection(
		double Ax, double Ay,
		double Bx, double By,
		double Cx, double Cy,
		double Dx, double Dy,
		double *X, double *Y);
		
	// True if the segment crosses the line defined by (p1, p2).
	// Stores the location of the intersection (if it exists) in "intersection"
	// This function approximates arcs by straight lines
	bool segIntersectLine(const PathSegment& seg, const Point& p1, const Point& p2, Point& intersection);
	
	
/* Path segment creation (create_seg.cpp) */
/*==============================================================================*/
	PathSegment
	makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2);
	
	PathSegment makeUndirectedLineSegment(double x1, double y1, double x2, double y2);
	// Makes a line segment respecting an initial angle (can handle backing up that way)
	PathSegment makeDirectedLineSegment(double x1, double y1, double x2, double y2, double t1);	
	
	PathSegment makeTurnSegment(double x, double y, double t1, double t2);
	PathSegment makeArcSegment(double x1, double y1, double x2, double y2, double t1, double t2);
	
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
	vector<PoseStamped> interpSegment(const PathSegment& seg, double dx, double dtheta);
	vector<PoseStamped> interpLineSegment(const PathSegment& seg, double dx);
	vector<PoseStamped> interpArcSegment(const PathSegment& seg, double dtheta);
	vector<PoseStamped> interpSpinSegment(const PathSegment& seg, double dtheta);
	
	PoseStamped getEndPose  (const PathSegment& seg);
	PoseStamped getStartPose(const PathSegment& seg);
	
	// Returns whether or not seg specifies backward motion
	bool isLineSegmentReversed(PathSegment seg);
	
};//namespace
#endif
