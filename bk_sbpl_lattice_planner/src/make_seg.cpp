#include <bk_sbpl_lattice_planner/make_seg.h>

namespace bk_sbpl_lattice_planner{
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
  const double pi = 3.1415926;
	double eps = .0001; // Precision for floating point comparison
	precision_navigation_msgs::PathSegment seg;
	double dx = x2-x1;
	double dy = y2-y1;
	t1 = rect_angle(t1);
	t2 = rect_angle(t2);
	double dth = t2-t1;
	fprintf(stdout,"t1 = %.2fpi, t2 = %.2fpi, dtheta = %.2fpi\n", t1/pi, t2/pi, dth/pi);
	
	// Arc parameters
	double chord_length, denom, signed_r, abs_r, ang_to_center, xcenter, ycenter;
	
	// Initialize the segment, set all speeds/accels to 0
	seg.max_speeds.linear.x  = 0;
	seg.max_speeds.angular.z = 0;
	seg.min_speeds.linear.x  = 0;
	seg.min_speeds.angular.z = 0;
	seg.accel_limit          = 0;
	seg.decel_limit          = 0;
	
	// No change in theta: line segment
	if(fabs(dth) < eps)
	{
		seg.seg_type       = precision_navigation_msgs::PathSegment::LINE;
		seg.seg_length     = sqrt(dx*dx + dy*dy);
		seg.ref_point.x    = x1;
		seg.ref_point.y    = y1;
		seg.ref_point.z    = 0.0;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
		seg.curvature      = 0.0;
		
		// Make sure the start/end angle is consistent with the path direction
		double expected_angle = rect_angle(atan2(dy,dx));
		if( fabs(expected_angle-t1) > eps ){
			ROS_ERROR("Path segment start/end angles were not consistent with line segment direction!");
		}
		
	}
	// No change in position: turn in place
	else if( fabs(dx)<eps && fabs(dy)<eps )
	{
		seg.seg_type       = precision_navigation_msgs::PathSegment::SPIN_IN_PLACE;
		seg.seg_length     = abs(dth);
		seg.ref_point.x    = x1;
		seg.ref_point.y    = y1;
		seg.ref_point.z    = 0.0;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
		
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
		
		// Use the formula chord_length = 2*radius*sin(dtheta) -> radius = chord_length/(2*sin(dtheta/2))
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
				ROS_ERROR("Invalid arc segment created!");
			}
		}
		else{
			seg.curvature = 0;
			ROS_ERROR("Invalid arc segment created!");
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
		
		fprintf(stdout,"dtheta        = %.2fpi\n"         , dth/pi);
		fprintf(stdout,"ang_to_center = %.2fpi\n"         , ang_to_center/pi);
		fprintf(stdout,"radius        = %.2f (%.2f)\n"    , signed_r, abs_r);
		fprintf(stdout,"chord_length  = %.2f\n"           , chord_length);
		fprintf(stdout,"arclength     = %.2f (%.2fpi)\n\n", seg.seg_length, seg.seg_length/pi);
	}
	
	return seg;
}

// DEBUG
void print_path_segment(precision_navigation_msgs::PathSegment s)
{
  const double pi = 3.1415926;
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
	fprintf(stdout,"Curvature:  %.2f\n", s.curvature);
	
}

// DEBUG
int main(int argc, char** argv)
{
  const double pi = 3.1415926;
	
	precision_navigation_msgs::PathSegment p;
	
	/*p = makePathSegment(0,0,pi/2,
	                    0,1,pi/2);
	*/
	/*p = makePathSegment(0.0, 0.0, pi/2.0,
	                   -1.0, 1.0, pi);
  */                   
	/*p = makePathSegment(0,0,pi/2,
	                    1,1,0);
	*/
	/*p = makePathSegment(0,0,pi/2,
	                    0,0,pi);
	*/
	/*p = makePathSegment(1,2,pi,
	                    -1,0,3*pi/2);
	*/
	
	/*p = makePathSegment(0,0,5*pi/4,
	                    -5,-5,5*pi/4);
	*/
	p = makePathSegment(-1,-1,3*pi/4,
	                    -1,1 ,pi/4);
	                    
	
	
	print_path_segment(p);
	
	return(0);
}
};
