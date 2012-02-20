#include <segment_lib/segment_lib.h>
namespace segment_lib{
	
// Returns a path segment between two points (x,y,theta)
// Note: initializes all speed/accel limits to 0
p_nav::PathSegment
makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2)
{
	
	double position_change = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
	double angle_change    = fabs(rect_angle(t2-t1));

	//ROS_INFO("t1 = %.2fpi, t2 = %.2fpi, dtheta = %.2fpi\n", t1/pi, t2/pi, dth/pi);

	// No change in theta: line segment
	if( angle_change < eps)
	{
		return makeDirectedLineSegment(x1, y1, x2, y2, t1);
	}
	// No change in position: turn in place
	else if( position_change < eps && angle_change > eps)
	{
		return makeTurnSegment(x1, y1, t1, t2);
	}
	// Else: arc segment
	else
	{
		return makeArcSegment(x1, y1, x2, y2, t1, t2);
	}
	
	return p_nav::PathSegment();
}

// Returns a line between two points without any initial angle
p_nav::PathSegment
makeUndirectedLineSegment(double x1, double y1, double x2, double y2)
{
	double expected_angle = rect_angle(atan2(y2-y1,x2-x1));
	return makeDirectedLineSegment(x1, y1, x2, y2, expected_angle);
}

// Makes a line segment respecting the general direction of an initial angle (can handle backing up that way)
p_nav::PathSegment
makeDirectedLineSegment(double x1, double y1, double x2, double y2, double t1)
{
	p_nav::PathSegment seg;

	// Initialize the segment, set all speeds/accels to 0
	seg.max_speeds.linear.x  = 0.0;
	seg.max_speeds.angular.z = 0.0;
	seg.min_speeds.linear.x  = 0.0;
	seg.min_speeds.angular.z = 0.0;
	seg.accel_limit          = 0.0;
	seg.decel_limit          = 0.0;
	
	double dx      = x2-x1;
	double dy      = y2-y1;
	double expected_angle = rect_angle(atan2(dy,dx));
	
	seg.seg_type       = p_nav::PathSegment::LINE;
	seg.seg_length     = sqrt(dx*dx + dy*dy);
	seg.ref_point.x    = x1;
	seg.ref_point.y    = y1;
	seg.ref_point.z    = 0.0;
	seg.curvature      = 0.0;
	
	// If there is a large discrepancy between the initial angle and the direction between the two points, assume the segment is specifying a reverse move
	double deviation = rect_angle(expected_angle-t1);
	
	//ROS_INFO("Deviation %.3fpi", deviation/pi);
	if( fabs(deviation) > pi/2 ){
		seg.seg_length    *= -1.0;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(expected_angle+pi);
		//ROS_INFO("Reverse seg detected");
	}
	else{
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(expected_angle);
	}
	
	if( fabs(seg.seg_length) < eps ){
		ROS_WARN("Degenerate line segment (length 0)");
	}else
	{
		// Make sure the start/end angle is consistent with the path direction
		/*double angle_error    = rect_angle(t1 - expected_angle);
		if( fabs(angle_error) > eps ){
			ROS_WARN("Path segment start/end angles were not consistent with line segment direction!");
			ROS_WARN("Calculated %.2fpi, but start=%.2fpi, end=%.2fpi",expected_angle/pi,t1/pi,t2/pi);
			ROS_WARN("%.2fpi off", rect_angle(angle_error));
		}*/
	}
	
	//ROS_INFO("Line (%.2f,%.2f)->(%.2f,%.2f) length %.2f",x1,y1,x2,y2,seg.seg_length);
	
	return seg;
}

p_nav::PathSegment
makeTurnSegment(double x, double y, double t1, double t2)
{
	p_nav::PathSegment seg;
	double dth = rect_angle(t2-t1);
	
	// Initialize the segment, set all speeds/accels to 0
	seg.max_speeds.linear.x  = 0.0;
	seg.max_speeds.angular.z = 0.0;
	seg.min_speeds.linear.x  = 0.0;
	seg.min_speeds.angular.z = 0.0;
	seg.accel_limit          = 0.0;
	seg.decel_limit          = 0.0;
	
	seg.seg_type       = p_nav::PathSegment::SPIN_IN_PLACE;
	seg.seg_length     = fabs(dth);
	seg.ref_point.x    = x;
	seg.ref_point.y    = y;
	seg.ref_point.z    = 0.0;
	seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
	
	// Positive curvature -> CW rotation.  Negative curvature -> CCW rotation
	if( dth>0 ){
		seg.curvature =  1.0;
	}else{
		seg.curvature = -1.0;
	}
		
	return seg;
}

p_nav::PathSegment
makeArcSegment(double x1, double y1, double x2, double y2, double t1, double t2)
{
	p_nav::PathSegment seg;
	double dth          = rect_angle(t2-t1);
	double dy           = y2-y1;
	double dx           = x2-x1;
	double chord_length = sqrt(dx*dx + dy*dy);
	
	// Initialize the segment, set all speeds/accels to 0
	seg.max_speeds.linear.x  = 0.0;
	seg.max_speeds.angular.z = 0.0;
	seg.min_speeds.linear.x  = 0.0;
	seg.min_speeds.angular.z = 0.0;
	seg.accel_limit          = 0.0;
	seg.decel_limit          = 0.0;
	
	seg.seg_type       = p_nav::PathSegment::ARC;
	seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
	
	// Find the angle between the starting tangent and the chord
	double dir_p1_p2       = rect_angle(atan2(dy, dx));
	double tan_chord_angle = fabs(rect_angle(dir_p1_p2-t1));
	
	// By geometry: the tangent-chord angle is twice the intercepted arc
	double arclen_theta    = 2.0*tan_chord_angle *dth/fabs(dth);
	
	// Use the formula chord_length = 2*radius*sin(dtheta/2) -> radius = chord_length/(2*sin(dtheta/2))
	// Note that from this formula, r can be either positive or negative (same as with curvature).
	// Positive r: segment bending to the left
	// Negative r: segment bending to the right
	double denom = 2*sin(arclen_theta/2.0);
	double signed_r, abs_r;
	if(denom != 0.0)
	{
		signed_r = chord_length / (denom);
		abs_r    = fabs(signed_r); // Absolute, positive radius
		
		if(signed_r != 0){
			seg.curvature = 1/signed_r;
		}else{
			seg.curvature = 0;
			ROS_WARN("Invalid arc segment (0 radius)!");
		}
	}
	else{
		seg.curvature = 0;
		ROS_WARN("Invalid arc segment (divide by 0)!");
	}
	
	// Find the angle to the circle's center from the starting point
	double ang_to_center;
	if( arclen_theta>0 )
		ang_to_center = rect_angle(t1 + pi/2); // 90 degrees to the left
	if( arclen_theta<0 )
		ang_to_center = rect_angle(t1 - pi/2); // 90 degrees to the right
	
	// Move to the circle's center
	double xcenter = x1 + abs_r*cos(ang_to_center);
	double ycenter = y1 + abs_r*sin(ang_to_center);
	
	seg.ref_point.x = xcenter;
	seg.ref_point.y = ycenter;
	seg.ref_point.z = 0.0;
	
	// Arc length = r*theta
	seg.seg_length = abs_r * fabs(arclen_theta);
	
	/*ROS_INFO("Dir p1->p2:      %.2fpi", dir_p1_p2/pi);
	ROS_INFO("tan_chord_angle: %.2fpi", tan_chord_angle/pi);
	ROS_INFO("arclen_theta:    %.2fpi", arclen_theta/pi);
	ROS_INFO("radius:          %.2f"  , signed_r); */
	/*ROS_INFO("dtheta        = %.2fpi\n"         , dth/pi);
	ROS_INFO("ang_to_center = %.2fpi\n"         , ang_to_center/pi);
	ROS_INFO("radius        = %.2f (%.2f)\n"    , signed_r, abs_r);
	ROS_INFO("chord_length  = %.2f\n"           , chord_length);
	ROS_INFO("arclength     = %.2f (%.2fpi)\n\n", seg.seg_length, seg.seg_length/pi);*/
	
	return seg;
}

};//namespace
