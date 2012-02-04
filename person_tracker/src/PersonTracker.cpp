#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <fstream>
#include <stdlib.h>
#include <stdio.h>

#define MAX_DIST (7.0)

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
static const char WINDOW[] = "Image window";
static const char WINDOW_EDGES[] = "Edges";
static const char WINDOW_HIST[] = "Histogram";

using std::cout;
using std::endl;
using std::string;


void doHistogram(const cv::Mat& image, cv::Mat& hist, cv::Mat& hist_image);
void doEM(const cv::Mat& image, CvEM& model, int num_clusters);

class SilhouetteTracker
{
	public:
		int var_;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		image_transport::SubscriberFilter image_sub_;
  	image_transport::Publisher  image_pub_;

		message_filters::Subscriber<PointCloudXYZRGB> cloud_sub_;
	
		SilhouetteTracker(int var);
		~SilhouetteTracker()
		{
			cv::destroyWindow(WINDOW);
			cv::destroyWindow(WINDOW_EDGES);
			cv::destroyWindow(WINDOW_HIST);
		}

	private:
		void bothCB(const sensor_msgs::ImageConstPtr& image_msg,
                const PointCloudXYZRGB::ConstPtr& cloud_msg);

		typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, PointCloudXYZRGB> MySyncPolicy;
		message_filters::Synchronizer< MySyncPolicy > sync_;
};


SilhouetteTracker::SilhouetteTracker(int var) : 
	nh_("silhouette_tracker"),	
	it_(nh_),
//	image_sub_( it_, "in_image", 1 ),
//	cloud_sub_( nh_, "in_cloud", 1 ),
	image_sub_( it_, "/camera/depth_registered/image_rect", 1),
	cloud_sub_( nh_, "/camera/depth_registered/points", 1 ),
	sync_( MySyncPolicy(2), image_sub_, cloud_sub_ )
{
	// publisher for the image
	std::string image_topic = nh_.resolveName("out_image");
  image_pub_ = it_.advertise(image_topic, 1);

	sync_.registerCallback( boost::bind( &SilhouetteTracker::bothCB, this, _1, _2) );
	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW_EDGES);
	cv::namedWindow(WINDOW_HIST);
	ROS_INFO("Silhouette tracker constructor finished");
}

int _filenum = 0;

void SilhouetteTracker::bothCB(const sensor_msgs::ImageConstPtr& image_msg, 
                               const PointCloudXYZRGB::ConstPtr& cloud_msg)
{
	ROS_WARN("Silhouette tracker got data, image format %s", image_msg->encoding.c_str());

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}

	//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	//	cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

	//cv::Mat img_blur;
	//cv::medianBlur( cv_ptr->image, img_blur, 5 );

	string filename("capture_01_30_");
	std::stringstream out;
	out << ++_filenum;
	filename += out.str();

	cout << filename << endl;

	std::ofstream myfile;
	myfile.open(filename.c_str());
	myfile << format(cv_ptr->image, "csv");
	myfile.close();
	cout << "Done!\n"; 

	//cv::Mat hist;
	//cv::Mat histImage;
	//doHistogram(img_blur/MAX_DIST, hist, histImage);
	
	//CvEM model;
	//doEM(img_blur/MAX_DIST, model, 10);

/*

	double minval, maxval;
	cv::Point minloc, maxloc;

	// Convert the depth image into 8-bit for edge detection, scaling by 255/MAX_DIST
	cv::Mat img_8bit;
	cv_ptr->image.convertTo(img_8bit, CV_8U, 255/((double)MAX_DIST), 0);

	cv::Mat src_blurred, detected_edges;

	//cv::medianBlur( img_8bit, detected_edges, 3 );

	int lowThreshold;
	int ratio;

	nh_.param<int>("canny/low_threshold", lowThreshold, 50);
	nh_.param<int>("canny/ratio"        , ratio       , 3 );
//	nh_.setParam("canny/ratio", 2);
	ROS_INFO("Low threshold=%d Ratio=%d",lowThreshold, ratio);
	
	int scale = 1;
	int delta = 0;
	int ddepth = CV_8U;
	cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, grad;
	cv::Sobel(img_8bit, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
	cv::Sobel(img_8bit, grad_y, ddepth, 1, 1, 3, scale, delta, cv::BORDER_DEFAULT);
  cv::convertScaleAbs( grad_x, abs_grad_x );
  cv::convertScaleAbs( grad_y, abs_grad_y );
	
  /// Total Gradient (approximate)
  cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
	
	cv::GaussianBlur(img_8bit, detected_edges, cv::Size(7,7), 3.0, 3.0);
	int kernel_size = 5;
	cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
	// Copy the detected edges to a new image, dst
	cv::Mat dst;
	dst.create(img_8bit.size(), img_8bit.type());
	dst = cv::Scalar::all(0);
	img_8bit.copyTo(dst, detected_edges);*/

	//cv::minMaxLoc(cv_ptr->image, &minval, &maxval, &minloc, &maxloc);

	//ROS_INFO("Max element = %f, min = %f", maxval, minval);

	// Convert to a fixed point image

	//cv::imshow(WINDOW, cv_ptr->image/MAX_DIST);
//	cv::imshow(WINDOW_EDGES, histImage);
	//cv::imshow(WINDOW_HIST, histImage);
	cv::waitKey(3);

  //image_pub_.publish(image_msg);
}


void doHistogram(const cv::Mat& image, cv::Mat& hist, cv::Mat& hist_image)
{

	/* Draw a histogram */
	int histSize = 50;
	float range[] = {0.0, 1};
	const float* histRange[] = {range};
	cv::calcHist( &image, 1, 0, cv::Mat() , hist, 1, &histSize, histRange, true, false );
	/// Normalize the result
	hist /= norm(hist, cv::NORM_L1);

	int hist_w = 400, hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );
	float sum=0;
	hist_image = cv::Mat( hist_w, hist_h, CV_8UC3, cv::Scalar( 0,0,0) );
	for( int i = 1; i < histSize; i++ )
	{
		//ROS_INFO("%d: %f",i,hist.at<float>(i-1));
		sum += hist.at<float>(i);
		line( hist_image, cv::Point( bin_w*(i-1), hist_h-cvRound(hist_h*hist.at<float>(i-1)) ) ,
		                 cv::Point( bin_w*(i)  , hist_h-cvRound(hist_h*hist.at<float>(i  )) ),
		                 cv::Scalar( 0, 0, 255), 2, 8, 0  );
	}
	//ROS_INFO("Sum = %f", sum);
}

void doEM(const cv::Mat& image, CvEM& model, int num_clusters)
{
	num_clusters = 5;
	int nd = 1;
	cv::Mat tempmat;

	
	// Take a subset of the image for training
	cv::Mat samples = image.clone();
	resize(samples, samples, cv::Size(4,4));
	int nsamples = samples.rows*samples.cols;

	samples = samples.reshape(1,nsamples/nd);
	cv::Mat training_set = samples.clone();

	// Get rid of NANs
	for( int i=0; i<nsamples; i++ ){
		if( cvIsNaN(training_set.at<float>(i)) ){
			training_set.at<float>(i) = 1.0;
			ROS_INFO("Corrected NAN");
		}
	}

	cv::Mat labels(nsamples, 1, CV_32SC1);

	ROS_INFO("%d samples",nsamples/nd);
	ROS_INFO("Image    size (%d,%d)", image.rows, image.cols); 
	ROS_INFO("Sample   size (%d,%d)", samples.rows, samples.cols); 

	cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 400, .0001);
	int attempts = 5;
	int flags = cv::KMEANS_PP_CENTERS;
	cv::Mat means(num_clusters, nd, CV_32FC1);

	ROS_INFO("Kmeans: %lf",kmeans(training_set, num_clusters, labels, criteria, attempts, flags, means));

	ROS_WARN("Training Done");
	
	ROS_INFO("Lbls  size (%d,%d)", labels.rows, labels.cols);
	ROS_INFO("Means size (%d,%d)", means.rows, means.cols);
	std::cout << "centers: " << means << std::endl;

	for( int i=0; i<nsamples; i++ ){
		std::cout << training_set.at<float>(i) << " -> " << labels.at<int>(i) << std::endl;
	}
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "silhouette_tracker");
	SilhouetteTracker st(3);
	ROS_INFO("Initialization done.");
  ros::spin();
	return(0);
}
