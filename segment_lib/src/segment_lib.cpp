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
void printPathSegment(const p_nav::PathSegment& s)
{
	switch(s.seg_type)
	{
		case p_nav::PathSegment::LINE:
			ROS_INFO("Line segment");
			break;
		case p_nav::PathSegment::SPIN_IN_PLACE:
			ROS_INFO("Spin in place");
			break;
		case p_nav::PathSegment::ARC:
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

// Reindexes the seg numbers.  Starts at start_seg_num.  Returns the last index used.
void
reindexPath(p_nav::Path& path, int start_seg_num)
{
	int seg_num = start_seg_num;
	
	for( unsigned int i=0; i<path.segs.size(); i++ )
	{
		path.segs.at(i).seg_number = seg_num;
		seg_num++;
	}
}

// Returns the index of seg_num within path.  -1 if DNE.
int segnumToIndex(const p_nav::Path& path, unsigned int seg_num)
{
	return segnumToIndex(path.segs, seg_num);
}

int
segnumToIndex(const std::vector<p_nav::PathSegment>& segs, unsigned int seg_num)
{
	// Try to find the segment with the given seg_num
	for( unsigned int i=0; i<segs.size(); i++ )
	{
		if( segs.at(i).seg_number == seg_num )
			return i;
	}
	
	// Didn't find it
	return -1;
}
	
int
getLastSegnum(const p_nav::Path& path)
{
	return path.segs.back().seg_number;
}
	
int
getFirstSegnum(const p_nav::Path& path)
{
	return path.segs.front().seg_number;
}


double
linDist(const p_nav::PathSegment& seg)
{
	/*if( seg.seg_type == SPIN_IN_PLACE ) {
		return 0.0;
	}*/
	
	return seg.seg_length;
}

void reFrame(p_nav::Path& path)
{
	p_nav::PathSegment seg;
	
	if( path.segs.size() > 0 )
	{
		seg = path.segs.back();
		path.header.frame_id = seg.header.frame_id;
		path.header.stamp    = seg.header.stamp;
		
		for( int i=0; i<path.segs.size()-1; i++ )
		{
			path.segs.at(i).header.frame_id = seg.header.frame_id;
			path.segs.at(i).header.stamp    = seg.header.stamp;
		}
		
	}
}


};//namespace
