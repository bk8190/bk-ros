#include <segment_lib/segment_lib.h>
#include <segment_lib/SegmentVisualizer.h>
#define makeseg  segment_lib::makePathSegment
#define printseg segment_lib::printPathSegment

int main(int argc, char** argv)
{
	ros::init(argc, argv, "segment_test_node");
	while( !ros::ok() ) { }
	ros::NodeHandle nh();
  const double pi = 3.1415926;
  
	precision_navigation_msgs::PathSegment seg;
	precision_navigation_msgs::Path        path;
  segment_lib::SegmentVisualizer         seg_vis("visualizer");
  
	ros::Duration(2.0).sleep();
	seg_vis.publishVisualization(path);
	
	

/*
	printf("(-1,-1,3/4p) -> (-1,1,1/4p)\n");
	seg = makeseg(-1,-1,3*pi/4,
	            -1, 1,pi/4);
	printseg(seg);
	
	printf("(-1,1,-3/4p) -> (-1,-1,-1/4p)\n");
	seg = makeseg(-1, 1,5*pi/4,
	            -1,-1,-pi/4);
	printseg(seg);
	
	printf("(1,-1,1/4p) -> (1,1,3/4p)\n");
	seg = makeseg(1,-1,pi/4,
	            1, 1,3*pi/4);
	printseg(seg);
	
	printf("(1,-1,1/4p) -> (1,1,4/4p) (bad)\n");
	seg = makeseg(1,-1,pi/4,
	            1, 1,pi);
	printseg(seg);
	
	printf("(1,0,1/2p) -> (-1,0,-1/2p)\n");
	seg = makeseg( 1.0/sqrt(2.0),1.0/sqrt(2.0),3*pi/4,
	            -1,0,-pi/2);
	printseg(seg);
*/
/*
	printf("(1,1,1/4p) -> (1,1,1/4p) (degenerate)\n");
	seg = makeseg( 1,1,1*pi/4,
	             1,1,1*pi/4);
	seg.header.frame_id = "map";

	printf("(1,1,1/4p) -> (6,6,1/4p) (bad angle)\n");
	seg = makeseg( 1,1,1*pi/4+.1,
	             6,6,1*pi/4+.1);
	printseg(seg);
	
	printf("(1,1,1/4p) -> (1,1,pi)\n");
	seg = makeseg( 1,1,pi/4,
	             1,1,pi);
	printseg(seg);*/
	
	ROS_INFO("Center (-1,-2) radius = 3, tangent 3pi/4 -> -pi/2");
	seg = makeseg(-1 + 3/sqrt(2), -2+3/sqrt(2), 3*pi/4,
	              -1-3, -2, -pi/2);
	seg.header.frame_id = "map";
	path.segs.push_back(seg);
	printseg(seg);
	
	/*
	ROS_INFO("Center (0,0) radius = 2, tangent -.75pi -> -.25pi");
	seg = makeseg(-2/sqrt(2), 2/sqrt(2), -.75*pi,
	              -2/sqrt(2),-2/sqrt(2), -.25*pi);
	seg.header.frame_id = "map";
	path.segs.push_back(seg);
	printseg(seg);
	
	/*
	ROS_INFO("Line (5,3) -> (-1,3)");
	seg = makeseg(5,3,pi,
	              -1,3,pi);
	seg.header.frame_id = "map";
	path.segs.push_back(seg);
	printseg(seg);
	
	
	ROS_INFO("Line (0,0) -> (4,4)");
	seg = makeseg(0,0,pi/4,
	              4,4,pi/4);
	seg.header.frame_id = "map";
	path.segs.push_back(seg);
	printseg(seg);*/
	
	seg_vis.publishVisualization(path);
	
	int i=0;
	while(ros::ok() && i++<2)
	{
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	return(0);
}
