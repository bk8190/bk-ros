#include <segment_lib/SegmentVisualizer.h>

namespace segment_lib {

SegmentVisualizer::SegmentVisualizer(std::string name):
	nh_         ( "/"+name),
	private_nh_ ("~/"+name),
	name_       (name),
	marker_uid_ (0)
{
	vis_markers_pub_  = nh_.advertise<visualization_msgs::MarkerArray> ("markers", 0);
	vis_path_pub_     = nh_.advertise<nav_msgs::Path> ("path", 0);
	vis_posearray_pub_= nh_.advertise<geometry_msgs::PoseArray> ("pose_array", 0);
	
	precision_navigation_msgs::Path empty_path;
	empty_path.header.frame_id = "map";
	empty_path.header.stamp    = ros::Time::now();
	publishVisualization(empty_path);
	
	marker_lifetime_ = ros::Duration(5.0);
	
	// Displaying markers is expensive for Rviz, so restrict them to a reasonable rate
	min_pub_interval_ = ros::Duration(0.5);
	last_pub_time_ = ros::Time::now();
}

SegmentVisualizer::~SegmentVisualizer()
{
	// Remove previous markers
	removePreviousMarkers();
	
	// Publish an empty path to clear current visualization
	publishPoseVisualization(nav_msgs::Path());
	publishPathVisualization(nav_msgs::Path());
}

void
SegmentVisualizer::publishVisualization(const precision_navigation_msgs::Path& path)
{
	ros::Time now = ros::Time::now();
	
	if( now - last_pub_time_ < min_pub_interval_ ) {
		return;
	}
	
	last_pub_time_ = now;
	
	precision_navigation_msgs::Path p2 = path;
	p2.header.stamp = now;
	
	for( unsigned int i=0; i<p2.segs.size(); i++ ) {
		p2.segs.at(i).header.stamp = now;
	}

	// Discretize the path
	nav_msgs::Path vis_path = interpPath(p2, .01, pi/32);
	
	// Publish markers path, and pose array visualizations.
	publishMarkerVisualization(p2);
	publishPathVisualization(vis_path);
	publishPoseVisualization(vis_path);
}

void
SegmentVisualizer::publishPathVisualization(const nav_msgs::Path& vis_path)
{
	// Just publish the path
	vis_path_pub_.publish(vis_path);
}

void
SegmentVisualizer::publishPoseVisualization(const nav_msgs::Path& vis_path)
{
	// Break up the path and publish the constituent poses
	geometry_msgs::PoseArray poses;
	
	// convert from posestamped to posearray
	for( int i=0; i<vis_path.poses.size(); i++ )
	{
		poses.poses.push_back(vis_path.poses.at(i).pose);
	}
	
	if( vis_path.poses.size() > 0 )
	{
		poses.header.stamp    = vis_path.poses.back().header.stamp;
		poses.header.frame_id = vis_path.poses.back().header.frame_id;
	}
	
	vis_posearray_pub_.publish(poses);
}


void
SegmentVisualizer::removePreviousMarkers()
{
	// If we've already sent markers, remove the last ones we've sent.
	// Do this by changing the command to "DELETE" and republishing.
	if( marker_uid_ > 0 )
	{
		for( unsigned int i=0; i<vis_markers_.markers.size(); i++ )
		{
			vis_markers_.markers.at(i).action = visualization_msgs::Marker::DELETE;
		}
		
		vis_markers_pub_.publish(vis_markers_);
	}
}


void SegmentVisualizer::publishMarkerVisualization(const precision_navigation_msgs::Path& path)
{
	removePreviousMarkers();

	char buffer[100];
	double perp_angle;
	vis_markers_.markers.clear();
	visualization_msgs::Marker m;
	std_msgs::ColorRGBA color;
				
	precision_navigation_msgs::PathSegment seg, last_seg;
	std::vector<geometry_msgs::PoseStamped> disc_seg;
	
	if( path.segs.size() > 0 ){
		last_seg.header = path.segs.back().header;
	}
	else {
		last_seg.header = path.header;
	}
	
	for( int i=0; i<path.segs.size(); i++ )
	{
		seg = path.segs.at(i);
		disc_seg = interpSegment(seg, 0.01, pi/64);
		
		if( disc_seg.size() == 0 ){
			ROS_WARN("Visualization interpolated a segment with no poses???");
			continue;
		}
		
		m = visualization_msgs::Marker();
		m.header.frame_id = last_seg.header.frame_id;
		m.header.stamp    = last_seg.header.stamp;
		m.ns              = name_;
		m.lifetime        = marker_lifetime_;
		m.action          = visualization_msgs::Marker::ADD;
		m.points.clear();
		m.colors.clear();
		
		switch(seg.seg_type)
		{
			case precision_navigation_msgs::PathSegment::LINE:
				//ROS_INFO("Line segment");
				
				// Add a label
				m.id    = ++marker_uid_;
				m.lifetime  = marker_lifetime_;
				m.type  = visualization_msgs::Marker::TEXT_VIEW_FACING;
				perp_angle = tf::getYaw(seg.init_tan_angle) + pi/2;
				m.pose.position.x = seg.ref_point.x + .05*cos(perp_angle);
				m.pose.position.y = seg.ref_point.y + .05*sin(perp_angle);
				m.pose.position.z = 0.3;
				m.scale.z = 0.03;
				m.color.a = 1.0;
				m.color.r = 1.0;
				m.color.g = 0.0;
				m.color.b = 0.5;
				sprintf(buffer,"%d: Line %.3fm\n", seg.seg_number, seg.seg_length);
				m.text = std::string(buffer);
				vis_markers_.markers.push_back(m);
				
				// Add a marker at the start point
				m.id              = ++marker_uid_;
				m.lifetime        = marker_lifetime_;
				m.type            = visualization_msgs::Marker::CYLINDER;
				m.pose.position.x = seg.ref_point.x;
				m.pose.position.y = seg.ref_point.y;
				m.pose.position.z = 0.0;
				
				m.scale.x = .005;
				m.scale.y = m.scale.x;
				m.scale.z = 1;
				m.color.a = 1.0;
				m.color.r = 1.0;
				m.color.g = 1.0;
				m.color.b = 1.0;
				vis_markers_.markers.push_back(m);
				
				
				// Add the line
				m.id      = ++marker_uid_;
				m.lifetime = marker_lifetime_;
				m.type    = visualization_msgs::Marker::LINE_STRIP;
				m.pose.position.x = 0.0;
				m.pose.position.y = 0.0;
				m.pose.position.z = 0.1;
				m.scale.x = 0.005;
				
				color.a = 1.0;
				color.r = 1.0;
				color.g = 0.0;
				color.b = 0.5;
				m.color = color;
				
				m.points.clear();
				m.colors.clear();
				
				for( int j=0; j<disc_seg.size(); j++ )
				{
					m.points.push_back(disc_seg.at(j).pose.position);
					m.colors.push_back(color);
				}
				vis_markers_.markers.push_back(m);
			
			break;
			
		case precision_navigation_msgs::PathSegment::SPIN_IN_PLACE:
				//ROS_INFO("Spin in place");
			
				// Add a label
				m.id    = ++marker_uid_;
				m.lifetime  = marker_lifetime_;
				m.type  = visualization_msgs::Marker::TEXT_VIEW_FACING;
				perp_angle = tf::getYaw(seg.init_tan_angle) + pi/2;
				m.pose.position.x = seg.ref_point.x + .09*cos(perp_angle);
				m.pose.position.y = seg.ref_point.y + .09*sin(perp_angle);
				m.pose.position.z = 0.3;
				m.scale.z = 0.03;
				
				m.color.a = 1.0;
				m.color.r = 0.9;
				m.color.g = 0.9;
				m.color.b = 0.0;
				sprintf(buffer,"%d: Spin %.3fpi\n", seg.seg_number, seg.seg_length/pi);
				m.text = std::string(buffer);
				vis_markers_.markers.push_back(m);
			
				// Add the vertex
				m.id              = ++marker_uid_;
				m.lifetime        = marker_lifetime_;
				m.type            = visualization_msgs::Marker::CYLINDER;
				m.pose.position.x = seg.ref_point.x;
				m.pose.position.y = seg.ref_point.y;
				m.pose.position.z = 0.0;
				
				m.scale.x = .01;
				m.scale.y = m.scale.x;
				m.scale.z = .2;	
				
				m.color.a = 1.0;
				m.color.r = 1.0;
				m.color.g = 1.0;
				m.color.b = 0.0;
				vis_markers_.markers.push_back(m);
				
				// Add direction arrows
				m.id        = ++marker_uid_;
				m.lifetime  = marker_lifetime_;
				m.type      = visualization_msgs::Marker::ARROW;
				m.pose      = disc_seg.front().pose;
				m.scale.x   = 1.0;
				m.scale.y   = 0.10;
				m.scale.z   = 0.10;
				m.color.a   = 1.0;
				m.color.r   = 1.0;
				m.color.g   = 0.0;
				m.color.b   = 0.0;
				vis_markers_.markers.push_back(m);
				
				m.id        = ++marker_uid_;
				m.lifetime  = marker_lifetime_;
				m.pose      = disc_seg.back().pose;
				m.scale.z  *= 0.8;
				m.color.r   = 0.0;
				m.color.g   = 1.0;
				vis_markers_.markers.push_back(m);
				
				
		break;
			
		case precision_navigation_msgs::PathSegment::ARC:
				//ROS_INFO("Arc segment");
				
				// Add a circle around the center
				/*m.id              = ++marker_uid_;
				m.lifetime           = marker_lifetime_;
				m.type            = visualization_msgs::Marker::CYLINDER;
				m.pose.position.x = seg.ref_point.x;
				m.pose.position.y = seg.ref_point.y;
				m.pose.position.z = 0.0;
				m.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				
				m.scale.x = 2*fabs(1/seg.curvature);
				m.scale.y = m.scale.x;
				m.scale.z = 0.01;	
				m.color.a = 0.03;
				m.color.r = 0.0;
				m.color.g = 0.0;
				m.color.b = 1.0;
				vis_markers_.markers.push_back(m);*/
			
				// Add a label
				m.id              = ++marker_uid_;
				m.lifetime        = marker_lifetime_;
				m.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
				perp_angle        = tf::getYaw(seg.init_tan_angle) + pi/2;
				m.pose.position.x = disc_seg.front().pose.position.x + .05*cos(perp_angle);
				m.pose.position.y = disc_seg.front().pose.position.y + .05*sin(perp_angle);
				m.pose.position.z = 0.3;
				
				m.scale.z = 0.03;
				m.color.a = 1.0;
				m.color.r = 0.3;
				m.color.g = 0.3;
				m.color.b = 1.0;
				sprintf(buffer,"%d: Arc\n", seg.seg_number);
				m.text = std::string(buffer);
				vis_markers_.markers.push_back(m);
			
				// Add the start point
				m.id              = ++marker_uid_;
				m.lifetime        = marker_lifetime_;
				m.type            = visualization_msgs::Marker::CYLINDER;
				m.pose.position.x = disc_seg.front().pose.position.x;
				m.pose.position.y = disc_seg.front().pose.position.y;
				m.pose.position.z = 0.0;
				
				m.scale.x = .005;
				m.scale.y = m.scale.x;
				m.scale.z = .5;	
				
				m.color.a = 1.0;
				m.color.r = 0.0;
				m.color.g = 1.0;
				m.color.b = 1.0;
				vis_markers_.markers.push_back(m);
				
			
				// Add the boundary line
				m.id              = ++marker_uid_;
				m.lifetime        = marker_lifetime_;
				m.type            = visualization_msgs::Marker::LINE_STRIP;
				m.pose.position.x = 0.0;
				m.pose.position.y = 0.0;
				m.pose.position.z = 0.2;
				m.scale.x = 0.005;
				m.scale.y = 0.005;
				m.scale.z = 0.005;
				
				color.a = 0.8;
				color.r = 0.0;
				color.g = 0.3;
				color.b = 1.0;
				m.color = color;
				
				m.points.clear();
				m.colors.clear();
				
				for( unsigned int j=0; j<disc_seg.size(); j++ )
				{
					m.points.push_back(disc_seg.at(j).pose.position);
					m.colors.push_back(color);
				}
				vis_markers_.markers.push_back(m);
			
			break;
			
		default:
			ROS_INFO("ERROR: Bad seg type");
		}
		
		vis_markers_pub_.publish(vis_markers_);
	}
}


};//namespace
