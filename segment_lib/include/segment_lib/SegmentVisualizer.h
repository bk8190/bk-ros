
#ifndef SEGMENT_VISUALIZER_H
#define SEGMENT_VISUALIZER_H

#include <segment_lib/segment_lib.h>

namespace segment_lib{

class SegmentVisualizer
{
	public:
		SegmentVisualizer(std::string name);
		~SegmentVisualizer();
	
		void publishVisualization(const precision_navigation_msgs::Path& path);
		
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		
		ros::Publisher  vis_markers_pub_;
		ros::Publisher  vis_path_pub_;
		ros::Publisher  vis_posearray_pub_;
		
		std::string name_;
		
		int marker_uid_;
		ros::Duration marker_lifetime_;
		
		ros::Duration min_pub_interval_;
		ros::Time     last_pub_time_;
		
		// Keep track of the last-sent markers
		visualization_msgs::MarkerArray vis_markers_;
		
		// Individually publishes markers, pose array, or discretized path for visualization
		void publishMarkerVisualization(const precision_navigation_msgs::Path& path);
		void publishPoseVisualization  (const nav_msgs::Path& vis_path);
		void publishPathVisualization  (const nav_msgs::Path& vis_path);
		
		// Sends a message to remove markers previously published
		void removePreviousMarkers();
};

};//namespace
#endif
