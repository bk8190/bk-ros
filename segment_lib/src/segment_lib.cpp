#include <segment_lib/segment_lib.h>

#include "interp_seg.cpp"  // Functions for turning segs into discrete arrays of poses
#include "smooth_path.cpp" // Functions for prettifying a segment path
#include "create_seg.cpp"  // Functions for creating segments

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

// DEBUG
void printPathSegment(const precision_navigation_msgs::PathSegment& s)
{
	switch(s.seg_type)
	{
		case precision_navigation_msgs::PathSegment::LINE:
			ROS_INFO("Line segment");
			break;
		case precision_navigation_msgs::PathSegment::SPIN_IN_PLACE:
			ROS_INFO("Spin in place");
			break;
		case precision_navigation_msgs::PathSegment::ARC:
			ROS_INFO("Arc segment");
			break;
		default:
			ROS_INFO("ERROR: Bad seg type");
	}
	
	ROS_INFO("\n");
	ROS_INFO("Length:     %.2f (%.2fpi)\n", s.seg_length, s.seg_length/pi);
	ROS_INFO("Ref point: (%.2f,%.2f)\n", s.ref_point.x, s.ref_point.y);
	ROS_INFO("Init angle: %.2fpi\n", tf::getYaw(s.init_tan_angle)/pi);
	ROS_INFO("Curvature:  %.2f\n\n", s.curvature);
}

};//namespace
