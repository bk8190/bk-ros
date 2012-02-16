#include <segment_lib/segment_lib.h>

namespace segment_lib {

// Takes in an angle, returns an equivalent angle in the range (-pi, pi)
double rect_angle(double t)
{/*
	while( t > pi )
		t -= 2*pi;
	while( t < -1.0*pi )
		t += 2*pi;	
	return t;*/
	return tf::getYaw(tf::createQuaternionMsgFromYaw(t));
}
// Returns a path segment between two points (x,y,theta)
// Note: initializes all speed/accel limits to 0
precision_navigation_msgs::PathSegment
makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2)
{
	double temp;
 	return makePathSegment(x1,y1,t1,x2,y2,t2,temp);
}
// Returns a path segment between two points (x,y,theta)
// Note: initializes all speed/accel limits to 0
precision_navigation_msgs::PathSegment
makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2, double& end_angle)
{
	double eps = .0001; // Precision for floating point comparison
	precision_navigation_msgs::PathSegment seg;
	
	double dx = (double)x2-(double)x1;
	double dy = (double)y2-(double)y1;
	t1 = rect_angle(t1);
	t2 = rect_angle(t2);
	double dth = rect_angle(t2-t1);
	
	//fprintf(stdout,"t1 = %.2fpi, t2 = %.2fpi, dtheta = %.2fpi\n", t1/pi, t2/pi, dth/pi);
	
	// Arc parameters
	double chord_length, denom, signed_r, abs_r, ang_to_center, xcenter, ycenter;
	
	// Initialize the segment, set all speeds/accels to 0
	seg.max_speeds.linear.x  = 0.0;
	seg.max_speeds.angular.z = 0.0;
	seg.min_speeds.linear.x  = 0.0;
	seg.min_speeds.angular.z = 0.0;
	seg.accel_limit          = 0.0;
	seg.decel_limit          = 0.0;
	
	double position_change = sqrt((double)dx*(double)dx + (double)dy*(double)dy);
	double angle_change    = fabs(dth);
	//ROS_INFO("pos change = %.3f ang change = %.3f", position_change, angle_change);
	//ROS_INFO("dx = %.2f dy = %.2f", dx, dy);
	
	// No change in theta: line segment
	if( angle_change < eps)
	{
		seg.seg_length        = sqrt(dx*dx + dy*dy);
		double expected_angle = rect_angle(atan2(dy,dx));
		
		//if( fabs(rect_angle(expected_angle-pi)) < eps ){
		//	seg.seg_length *= -1.0;
		//}
		
		seg.seg_type       = precision_navigation_msgs::PathSegment::LINE;
		seg.ref_point.x    = x1;
		seg.ref_point.y    = y1;
		seg.ref_point.z    = 0.0;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(expected_angle);
		seg.curvature      = 0.0;
		
		if( fabs(seg.seg_length) < eps )
			ROS_WARN("Degenerate line segment (length 0)");
		else
		{
			// Make sure the start/end angle is consistent with the path direction
			/*double angle_error    = rect_angle(t1 - expected_angle);
			if( fabs(angle_error) > eps ){
				ROS_WARN("Path segment start/end angles were not consistent with line segment direction!");
				ROS_WARN("Calculated %.2fpi, but start=%.2fpi, end=%.2fpi",expected_angle/pi,t1/pi,t2/pi);
				ROS_WARN("%.2fpi off", rect_angle(angle_error));
			}*/
		}
		
		end_angle = expected_angle;
		
	}
	// No change in position: turn in place
	else if( position_change < eps && angle_change > eps)
	{
		seg.seg_type       = precision_navigation_msgs::PathSegment::SPIN_IN_PLACE;
		seg.seg_length     = fabs(dth);
		seg.ref_point.x    = x1;
		seg.ref_point.y    = y1;
		seg.ref_point.z    = 0.0;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
		
		end_angle = t2;
		
		// Positive curvature -> CW rotation.  Negative curvature -> CCW rotation
		if( dth>0 )
			seg.curvature =  1.0;
		else
			seg.curvature = -1.0;
	}
	// Else: arc segment
	else
	{
		seg.seg_type = precision_navigation_msgs::PathSegment::ARC;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
		
		// Use the formula chord_length = 2*radius*sin(dtheta/2) -> radius = chord_length/(2*sin(dtheta/2))
		// Note that from this formula, r can be either positive or negative (same as with curvature).
		// Positive r: segment bending to the left
		// Negative r: segment bending to the right
		chord_length = sqrt(dx*dx + dy*dy);
		denom = 2*sin(dth/2);
		if(denom != 0)
		{
			signed_r = chord_length / (denom);
			abs_r    = fabs((double)signed_r); // Absolute, positive radius
			
			if(signed_r != 0)		
				seg.curvature = 1/signed_r;
			else{
				seg.curvature = 0;
				ROS_WARN("Invalid arc segment (0 radius)!");
			}
		}
		else{
			seg.curvature = 0;
			ROS_WARN("Invalid arc segment (divide by 0)!");
		}
		
		// Find the angle to the circle's center from the starting point
		if( dth>0 )
			ang_to_center = rect_angle(t1 + pi/2); // 90 degrees to the left
		if( dth<0 )
			ang_to_center = rect_angle(t1 - pi/2); // 90 degrees to the right
		
		// Move to the circle's center
		xcenter = x1 + abs_r*cos((double)ang_to_center);
		ycenter = y1 + abs_r*sin((double)ang_to_center);
		
		seg.ref_point.x = xcenter;
		seg.ref_point.y = ycenter;
		seg.ref_point.z = 0.0;
		
		// Arc length = r*theta
		seg.seg_length = abs_r * fabs((double)dth);
		
		// Perform a consistency check.  From the circle's center, navigate to the end point.
		double ang_center_to_end = ang_to_center+dth+pi;
		double x2_predicted = xcenter + abs_r*cos((double)(ang_center_to_end));
		double y2_predicted = ycenter + abs_r*sin((double)(ang_center_to_end));
		
		// Make sure the calculated endpoint is consistent with the actual endpoint
		double deviation = abs(x2 - x2_predicted) + abs(y2 - y2_predicted);
		if( deviation > eps ){
			ROS_WARN("Arc segment with inconsistent endpoint angles: Deviation=%.5f",deviation);
		}
		
		ROS_INFO("Arc from (%.2f,%.2f)->(%.2f,%.2f)",x1,y1,x2_predicted,y2_predicted);
		end_angle = t2;
		
		/*fprintf(stdout,"dtheta        = %.2fpi\n"         , dth/pi);
		fprintf(stdout,"ang_to_center = %.2fpi\n"         , ang_to_center/pi);
		fprintf(stdout,"radius        = %.2f (%.2f)\n"    , signed_r, abs_r);
		fprintf(stdout,"chord_length  = %.2f\n"           , chord_length);
		fprintf(stdout,"arclength     = %.2f (%.2fpi)\n\n", seg.seg_length, seg.seg_length/pi);*/
	}
	
	return seg;
}

// DEBUG
void printPathSegment(const precision_navigation_msgs::PathSegment& s)
{
	switch(s.seg_type)
	{
		case precision_navigation_msgs::PathSegment::LINE:
			fprintf(stdout,"Line segment");
			break;
		case precision_navigation_msgs::PathSegment::SPIN_IN_PLACE:
			fprintf(stdout,"Spin in place");
			break;
		case precision_navigation_msgs::PathSegment::ARC:
			fprintf(stdout,"Arc segment");
			break;
		default:
			fprintf(stdout,"ERROR: Bad seg type");
	}
	
	fprintf(stdout,"\n");
	fprintf(stdout,"Length:     %.2f (%.2fpi)\n", s.seg_length, s.seg_length/pi);
	fprintf(stdout,"Ref point: (%.2f,%.2f)\n", s.ref_point.x, s.ref_point.y);
	fprintf(stdout,"Init angle: %.2fpi\n", tf::getYaw(s.init_tan_angle)/pi);
	fprintf(stdout,"Curvature:  %.2f\n\n", s.curvature);
}

// Returns a vector of poses sampled from a segment path (same as calling interpSegment multiple times and concatenating)
nav_msgs::Path
interpPath(const precision_navigation_msgs::Path& path, double dx, double dtheta)
{
	nav_msgs::Path interp_path;
	std::vector<geometry_msgs::PoseStamped> newpoints;
	
	// Interpolate each segment and concatenate to the path
	for( unsigned int i=0; i<path.segs.size(); i++ ) {
		newpoints = interpSegment(path.segs.at(i), dx, dtheta);
		interp_path.poses.insert( interp_path.poses.end(), newpoints.begin(), newpoints.end() );
	}
	
	// Fill out the path's header
	if( path.segs.size() > 0 ) {
		interp_path.header.stamp    = path.segs.at(0).header.stamp;
		interp_path.header.frame_id = path.segs.at(0).header.frame_id;
	}
	
	return interp_path;
}


// Returns a vector of poses from a segment: the beginning, the end, and regular samples along distance dx (line/arc) and dtheta (spin in place).
std::vector<geometry_msgs::PoseStamped> interpSegment(const precision_navigation_msgs::PathSegment& seg, double dx, double dtheta)
{
	switch(seg.seg_type)
	{
		case precision_navigation_msgs::PathSegment::LINE :
			return interpLineSegment(seg, dx);
			break;
		
		case precision_navigation_msgs::PathSegment::ARC :
			return interpArcSegment(seg, dtheta);
			break;
			
		case precision_navigation_msgs::PathSegment::SPIN_IN_PLACE :
			return interpSpinSegment(seg, dtheta);
			break;
			
		default :
			ROS_ERROR("Interpolate called on an unknown segment type");
			std::vector<geometry_msgs::PoseStamped> points;
			return points;
	}
}

std::vector<geometry_msgs::PoseStamped>
interpLineSegment(const precision_navigation_msgs::PathSegment& seg, double dx)
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
		p.header.frame_id = seg.header.frame_id;
		points.push_back(p);
		
		traversed_length += dx;
	}
	
	// Add the endpoint
	p.pose.position.x  = x0 + seg.seg_length*cos(tan_angle);
	p.pose.position.y  = y0 + seg.seg_length*sin(tan_angle);
	p.pose.position.z  = 0.0;
	p.pose.orientation = seg.init_tan_angle;
	p.header.frame_id = seg.header.frame_id;
	points.push_back(p);
	
	return points;
}

std::vector<geometry_msgs::PoseStamped>
interpArcSegment(const precision_navigation_msgs::PathSegment& seg, double dtheta)
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
	points.push_back(p);
	
	return points;
}

std::vector<geometry_msgs::PoseStamped>
interpSpinSegment(const precision_navigation_msgs::PathSegment& seg, double dtheta)
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
	points.push_back(p);
	
	return points;
}

};//namespace
