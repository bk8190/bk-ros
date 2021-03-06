#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_decimator/PCLDecimatorConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/radius_outlier_removal.h"

#include <string>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher cloudout_pub_, cloudout_old_pub_;

std::string fieldName_ = "z";
float startPoint_ = .4;
float endPoint_ = 1.0;
float leafSize_ = 0.02;
float outlierRadius_ = 0.2;
int   numInRadius_ = 5;

tf::TransformListener* tfl;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

// CALLBACKS
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_raw)
{
	sensor_msgs::PointCloud2::Ptr cloud_transformed (new sensor_msgs::PointCloud2());
	sensor_msgs::PointCloud2::Ptr cloud_cropped     (new sensor_msgs::PointCloud2());
	sensor_msgs::PointCloud2::Ptr cloud_inliers     (new sensor_msgs::PointCloud2());
	sensor_msgs::PointCloud2::Ptr cloud_downsampled (new sensor_msgs::PointCloud2());
	
	// Transform the pointcloud
	try{
		ROS_DEBUG_STREAM("Transforming");
		pcl_ros::transformPointCloud( "base_link", *cloud_raw, *cloud_transformed, *tfl);
	} catch( tf::TransformException ex ){
		ROS_WARN_STREAM("PCL decimator could not transform: " << ex.what() );
		return;
	}

	ros::Time t1 = ros::Time::now();	

	ROS_DEBUG_STREAM("Filtering");
	// Create a pass-through filter
	pcl::PassThrough<sensor_msgs::PointCloud2> pass;
	pass.setInputCloud      (cloud_transformed );
	pass.setFilterFieldName (fieldName_        );
	pass.setFilterLimits    (startPoint_, endPoint_);
	pass.filter             (*cloud_cropped    );
	
	// Now create a voxel grid filter
	pcl::VoxelGrid<sensor_msgs::PointCloud2> vgrid;
	vgrid.setInputCloud     (cloud_cropped      );
	vgrid.setLeafSize       (leafSize_, leafSize_, leafSize_);
	vgrid.filter            (*cloud_downsampled );
	
	// Remove outliers
	pcl::RadiusOutlierRemoval<sensor_msgs::PointCloud2> radius;
	radius.setInputCloud    (cloud_downsampled     );
	radius.setRadiusSearch  (outlierRadius_    );
	radius.setMinNeighborsInRadius(numInRadius_);
	radius.filter           (*cloud_inliers    );


	ros::Duration d = ros::Time::now() - t1;
	ROS_DEBUG_STREAM("Filtering took " << (d.toSec()) << " seconds.");

	cloudout_pub_.publish   (*cloud_inliers);
	
	// Convert to 
	sensor_msgs::PointCloud old_cloud;
	sensor_msgs::convertPointCloud2ToPointCloud(*cloud_downsampled, old_cloud);
	cloudout_old_pub_.publish(old_cloud);
	
	
	ros::Duration(0.5).sleep();
}



void callback(pcl_decimator::PCLDecimatorConfig &config, uint32_t level)
{
  fieldName_  = config.field_name;
  startPoint_ = config.start_threshold;
  endPoint_   = config.end_threshold;
  leafSize_   = config.leaf_size;
  numInRadius_= config.num_in_radius;
  outlierRadius_ = config.outlier_radius;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_decimator");
  ros::NodeHandle n;
  tfl = new tf::TransformListener();
  
  // Allow the transform listner some time to receive messages
  ros::Duration(3).sleep();

  cloudout_pub_     = n.advertise< sensor_msgs::PointCloud2 >("/camera/depth/points_sliced"    , 1);
  cloudout_old_pub_ = n.advertise< sensor_msgs::PointCloud  >("/camera/depth/points_sliced_old", 1);

  ros::Subscriber kinect_sub = n.subscribe("/camera/depth/points", 1, pointcloudCallback);
  
  dynamic_reconfigure::Server<pcl_decimator::PCLDecimatorConfig> srv;
  dynamic_reconfigure::Server<pcl_decimator::PCLDecimatorConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);

  ros::spin();

  delete tfl;
  return 0;
}




