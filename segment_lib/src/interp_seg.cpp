#include <segment_lib/segment_lib.h>
namespace segment_lib{

// Returns a vector of poses sampled from a segment path (same as calling interpSegment multiple times and concatenating the results)
nav_msgs::Path
interpPath(const p_nav::Path& path, double dx, double dtheta)
{
	nav_msgs::Path interp_path;
	std::vector<geometry_msgs::PoseStamped> newpoints;
	
	// Fill out the path's header
	if( path.segs.size() > 0 ){
		interp_path.header.stamp    = path.segs.back().header.stamp;
		interp_path.header.frame_id = path.segs.back().header.frame_id;
	}else {
		interp_path.header.stamp    = path.header.stamp;
		interp_path.header.frame_id = path.header.frame_id;
	}
	
	// Interpolate each segment and concatenate to the path
	for( unsigned int i=0; i<path.segs.size(); i++ ) {
		newpoints = interpSegment(path.segs.at(i), dx, dtheta);
		interp_path.poses.insert( interp_path.poses.end(), newpoints.begin(), newpoints.end() );
	}
	
	return interp_path;
}

	
// Returns a vector of poses from a segment: the beginning, the end, and regular samples along distance dx (line/arc) and dtheta (spin in place).
std::vector<geometry_msgs::PoseStamped> interpSegment(const p_nav::PathSegment& seg, double dx, double dtheta)
{
	switch(seg.seg_type)
	{
		case p_nav::PathSegment::LINE :
			return interpLineSegment(seg, dx);
			break;
		
		case p_nav::PathSegment::ARC :
			return interpArcSegment(seg, dtheta);
			break;
			
		case p_nav::PathSegment::SPIN_IN_PLACE :
			return interpSpinSegment(seg, dtheta);
			break;
			
		default :
			ROS_ERROR("Interpolate called on an unknown segment type");
			// points;
			return std::vector<geometry_msgs::PoseStamped>();
	}
}

std::vector<geometry_msgs::PoseStamped>
interpLineSegment(const p_nav::PathSegment& seg, double dx)
{
	std::vector<geometry_msgs::PoseStamped> points;
	geometry_msgs::PoseStamped p;
	
	if( dx <= 0.0 ){
		ROS_ERROR("Interpolate called with negative/zero step");
		return points;
	}
	
	double x0 = seg.ref_point.x;
	double y0 = seg.ref_point.y;
	double tan_angle = tf::getYaw(seg.init_tan_angle);
	double traversed_length = 0.0;
	
	// Add the beginning point and intermediate points
	while(fabs(traversed_length) < fabs(seg.seg_length))
	{
		p.pose.position.x  = x0 + traversed_length*cos(tan_angle);
		p.pose.position.y  = y0 + traversed_length*sin(tan_angle);
		p.pose.position.z  = 0.0;
		p.pose.orientation = seg.init_tan_angle;
		p.header.frame_id  = seg.header.frame_id;
		p.header.stamp     = seg.header.stamp;
		points.push_back(p);
		
		traversed_length += dx;
	}
	
	// Add the endpoint
	p.pose.position.x  = x0 + fabs(seg.seg_length)*cos(tan_angle);
	p.pose.position.y  = y0 + fabs(seg.seg_length)*sin(tan_angle);
	p.pose.position.z  = 0.0;
	p.pose.orientation = seg.init_tan_angle;
	p.header.frame_id  = seg.header.frame_id;
	p.header.stamp     = seg.header.stamp;
	points.push_back(p);
	
	return points;
}

std::vector<geometry_msgs::PoseStamped>
interpArcSegment(const p_nav::PathSegment& seg, double dtheta)
{
	std::vector<geometry_msgs::PoseStamped> points;
	geometry_msgs::PoseStamped p;
	
	if( dtheta <= 0.0 ){
		ROS_ERROR("Interpolate called with negative/zero step");
		return points;
	}
	if( fabs(seg.curvature) <= 0 ){
		ROS_ERROR("Interpolate called on an arc with zero curvature");
		return points;
	}
	
	double theta0 = tf::getYaw(seg.init_tan_angle); // initial tangent angle
	double radius = 1.0/fabs(seg.curvature);
	double arclen = fabs(seg.seg_length);
	double xc     = seg.ref_point.x;
	double yc     = seg.ref_point.y;
	
	// From the formula arclen = r*theta -> theta = arclen/r
	double arclen_theta    = arclen/radius; // Total change in angle
	double traversed_angle = 0.0;           // Current progress along the arc (in radians)
	
	double curr_tan_angle;    // Current tangent angle on the arc
	double ang_center_to_pt;  // Direction angle from circle's center to current point on the arc
	
	/*ROS_INFO("Arclen = %.2f (angular %.2fpi)", arclen, arclen_theta/pi);
	ROS_INFO("theta0 = %.2fpi" , theta0/pi);
	ROS_INFO("dtheta = %.2fpi\n" , dtheta/pi);*/
	
	// Traverse the arc in increments of dtheta
	while(fabs(traversed_angle) < arclen_theta)
	{
		// Tangent to the arc at this point
		curr_tan_angle = rect_angle(theta0 + traversed_angle);
		
		// Direction angle from circle's center to current point on the arc
		if( seg.curvature > 0 )
			ang_center_to_pt = rect_angle(curr_tan_angle - pi/2);
		else
			ang_center_to_pt = rect_angle(curr_tan_angle + pi/2);
			
		// Navigate from the center to the point on the arc
		p.pose.position.x  = xc + radius*cos(ang_center_to_pt);
		p.pose.position.y  = yc + radius*sin(ang_center_to_pt);
		p.pose.position.z  = 0.0;
		p.pose.orientation = tf::createQuaternionMsgFromYaw(curr_tan_angle);
		p.header.frame_id = seg.header.frame_id;
		p.header.stamp    = seg.header.stamp;
		points.push_back(p);
		
		/*ROS_INFO("Traversed %.2fpi rads.", traversed_angle/pi);
		ROS_INFO("Tangent   %.2fpi, dir from center = %.2fpi", curr_tan_angle/pi, ang_center_to_pt/pi);
		ROS_INFO("Point coords (%.2f,%.2f)\n", p.pose.position.x, p.pose.position.y);*/
		
		// Positive curvature -> CCW, Negative curvature -> CW
		if( seg.curvature > 0 )
			traversed_angle += dtheta;
		else
			traversed_angle -= dtheta;
	}
	
	// Add the endpoint
	
	// Positive curvature -> CCW, Negative curvature -> CW
	if( seg.curvature > 0 )
		curr_tan_angle =  rect_angle(theta0 + arclen_theta);
	else
		curr_tan_angle =  rect_angle(theta0 - arclen_theta);
	
	// Direction angle from the circle center to current point on the arc
	if( seg.curvature > 0 )
		ang_center_to_pt = rect_angle(curr_tan_angle - pi/2);
	else
		ang_center_to_pt = rect_angle(curr_tan_angle + pi/2);
		
	// Navigate from the center to the point on the arc
	p.pose.position.x  = xc + radius*cos(ang_center_to_pt);
	p.pose.position.y  = yc + radius*sin(ang_center_to_pt);
	p.pose.position.z  = 0.0;
	p.pose.orientation = tf::createQuaternionMsgFromYaw(curr_tan_angle);
	p.header.frame_id = seg.header.frame_id;
	p.header.stamp    = seg.header.stamp;
	points.push_back(p);
	
	return points;
}

std::vector<geometry_msgs::PoseStamped>
interpSpinSegment(const p_nav::PathSegment& seg, double dtheta)
{
	std::vector<geometry_msgs::PoseStamped> points;
	geometry_msgs::PoseStamped p;
	
	if( dtheta <= 0.0 ){
		ROS_ERROR("Interpolate called with negative/zero step");
		return points;
	}
	
	double theta0 = tf::getYaw(seg.init_tan_angle);
	double traversed_angle = 0.0;
	
	while(fabs(traversed_angle) < fabs(seg.seg_length))
	{
		p.pose.position.x = seg.ref_point.x;
		p.pose.position.y = seg.ref_point.y;
		p.pose.position.z = 0.0;
		p.pose.orientation = tf::createQuaternionMsgFromYaw(theta0 + traversed_angle);
		p.header.frame_id = seg.header.frame_id;
		p.header.stamp    = seg.header.stamp;
		points.push_back(p);
		
		// Positive curvature -> CCW
		if( seg.curvature > 0 ){
			traversed_angle += dtheta;
		}
		// Negative curvature -> CW
		else{
			traversed_angle -= dtheta;
		}
	}
	
	// Add the endpoint
		
	// Positive curvature -> CCW
	if( seg.curvature > 0 ){
		traversed_angle =  fabs(seg.seg_length);
	}
	// Negative curvature -> CW
	else{
		traversed_angle = -fabs(seg.seg_length);
	}
	
	p.pose.position.x = seg.ref_point.x;
	p.pose.position.y = seg.ref_point.y;
	p.pose.position.z = 0.0;
	p.pose.orientation = tf::createQuaternionMsgFromYaw(theta0 + traversed_angle);
	p.header.frame_id = seg.header.frame_id;
	p.header.stamp    = seg.header.stamp;
	points.push_back(p);
	
	return points;
}

// Returns whether or not seg specifies backward motion
bool isLineSegmentReversed(p_nav::PathSegment seg)
{
	return( seg.seg_length < 0 );
}

geometry_msgs::PoseStamped
getEndPose(const p_nav::PathSegment& seg)
{
	std::vector<geometry_msgs::PoseStamped> interp = interpSegment(seg, 1.0, 1.0);
	return interp.back();
}

geometry_msgs::PoseStamped
getStartPose(const p_nav::PathSegment& seg)
{
	std::vector<geometry_msgs::PoseStamped> interp = interpSegment(seg, 1.0, 1.0);
	return interp.front();
}

}//namespace
