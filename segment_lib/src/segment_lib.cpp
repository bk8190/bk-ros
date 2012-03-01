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
	if( seg.seg_type == p_nav::PathSegment::SPIN_IN_PLACE )
	{
		return seg.seg_length*1.0;
	}

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

//  public domain function by Darel Rex Finley, 2006
//  Determines the intersection point of the line segment defined by points A and B
//  with the line segment defined by points C and D.
//
//  Returns YES if the intersection point was found, and stores that point in X,Y.
//  Returns NO if there is no determinable intersection point, in which case X,Y will
//  be unmodified.
bool lineSegmentIntersection(
		double Ax, double Ay,
		double Bx, double By,
		double Cx, double Cy,
		double Dx, double Dy,
		double *X, double *Y)
{
	double  distAB, theCos, theSin, newX, ABpos;

	//  Fail if either line segment is zero-length.
	if (Ax==Bx && Ay==By || Cx==Dx && Cy==Dy) { return false; }

	//  Fail if the segments share an end-point.
	if (Ax==Cx && Ay==Cy || Bx==Cx && By==Cy
	||  Ax==Dx && Ay==Dy || Bx==Dx && By==Dy) {	return false; }

	//  (1) Translate the system so that point A is on the origin.
	Bx-=Ax; By-=Ay;
	Cx-=Ax; Cy-=Ay;
	Dx-=Ax; Dy-=Ay;

	//  Discover the length of segment A-B.
	distAB=sqrt(Bx*Bx+By*By);

	//  (2) Rotate the system so that point B is on the positive X axis.
	theCos=Bx/distAB;
	theSin=By/distAB;
	newX=Cx*theCos+Cy*theSin;
	Cy  =Cy*theCos-Cx*theSin; Cx=newX;
	newX=Dx*theCos+Dy*theSin;
	Dy  =Dy*theCos-Dx*theSin; Dx=newX;

	//  Fail if segment C-D doesn't cross line A-B.
	if (Cy<0. && Dy<0. || Cy>=0. && Dy>=0.) { return false; }

	//  (3) Discover the position of the intersection point along line A-B.
	ABpos=Dx+(Cx-Dx)*Dy/(Dy-Cy);

	//  Fail if segment C-D crosses line A-B outside of segment A-B.
	if (ABpos<0. || ABpos>distAB) { return false; }

	//  (4) Apply the discovered position to line A-B in the original coordinate system.
	*X=Ax+ABpos*theCos;
	*Y=Ay+ABpos*theSin;

	//  Success.
	return true;
}

// True if the segment crosses the line defined by (p1, p2).
// Stores the location of the intersection (if it exists) in "intersection"
// This function approximates arcs by straight lines
bool segIntersectLine(const PathSegment& seg, const Point& p1, const Point& p2, Point& intersection)
{
	double Ax, Bx, Cx, Dx, Ay, By, Cy, Dy;
	
	PoseStamped start = getStartPose(seg);
	PoseStamped end   = getEndPose(seg);
	Ax = start.pose.position.x;
	Ay = start.pose.position.y;
	Bx = end  .pose.position.x;
	By = end  .pose.position.y;
	
	Cx = p1.x;
	Cy = p1.y;
	Dx = p2.x;
	Dy = p2.y;
	
	return lineSegmentIntersection(Ax,Ay,Bx,By,Cx,Cy,Dx,Dy, &intersection.x, &intersection.y);
}


};//namespace
