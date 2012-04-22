/*****************************************************************************
*                                                                            *
*  OpenNI 1.0 Alpha                                                          *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <vector>
#include <map>

#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <people_msgs/PositionMeasurement.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

//#include "opencv/cxcore.hpp"
#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "PersonCal.cpp"
//---------------------------------------------------------------------------
// OpenNI includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnTypes.h>
#include "SceneDrawer.h"

#include <GL/glut.h>

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480

using std::string;
using std::vector;
using std::pair;
using std::map;
const double BIGDIST_M = 1000000.0;
//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;
xn::ImageGenerator g_ImageGenerator;

// Is a pose required for calibration? If so, g_strPose holds its name.
XnBool g_bNeedPose   = false;
XnChar g_strPose[20] = "";

// Have we successfully calibrated a user? If so, which user?
XnBool      g_bhasCal = false;
XnUserID    first_calibrated_user_;
double smoothing_factor_;
PersonCal user_cal_, original_cal_;

// Controls what is drawn to the screen
XnBool g_bDrawBackground = true;
XnBool g_bDrawPixels     = true;
XnBool g_bDrawSkeleton   = true;
XnBool g_bPrintID        = true;
XnBool g_bPrintState     = true;
XnBool g_bPause          = false;

// ROS stuff
ros::Publisher cloud_pub_, pos_pub_, has_lock_pub_;
std::string    frame_id_;
tf::TransformListener* tfl_;

double pub_rate_temp_;
image_geometry::PinholeCameraModel cam_model_;
bool got_cam_info_;
cv::Mat latest_rgb_;
bool got_rgb_;
double rgb_shift_h, rgb_shift_v, rgb_scale_z;

double association_dist_, min_dist_, min_area_, max_area_, reliability_;
double variance_xy_;
boost::mutex pos_mutex_;
//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
struct RestampedPositionMeasurement
{
	ros::Time restamp;
	people_msgs::PositionMeasurement pos;
	double dist;
};
pair<string, RestampedPositionMeasurement> latest_tracker_;

void cam_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	if( cam_model_.fromCameraInfo(msg) )
	{
		got_cam_info_ = true;
		ROS_INFO("[bk_skeletal_tracker] Got RGB camera info.");
	} else {
		ROS_ERROR("[bk_skeletal_tracker] Couldn't read camera info.");
	}
}

void posCallback(const people_msgs::PositionMeasurementConstPtr& pos_ptr)
{
	boost::mutex::scoped_lock pos_lock(pos_mutex_);
	
	string msg = str(boost::format("[bk_skeletal_tracker] Position measurement \"%s\" (%.2f,%.2f) - ")
	            % pos_ptr->object_id.c_str() % pos_ptr->pos.x % pos_ptr->pos.y );
	                 
	RestampedPositionMeasurement rpm;
	rpm.pos     = *pos_ptr;
	rpm.restamp = pos_ptr->header.stamp;
	rpm.dist    = BIGDIST_M;
	
	// Put the incoming position into the position queue. It'll be processed in the next image callback.
	bool found = latest_tracker_.first == pos_ptr->object_id;
	if (!found) {
		latest_tracker_ = pair<string, RestampedPositionMeasurement>(pos_ptr->object_id, rpm);
		msg += "New object";
		ROS_DEBUG_STREAM(msg);
	}
	else {
		latest_tracker_.second = rpm;
		msg += "Existing object";
		ROS_DEBUG_STREAM(msg);
	}
}

struct user
{
	PersonCal pc;
	
	geometry_msgs::PointStamped center3d;
	XnUserID uid;
	double distance;
	
	int    numpixels;
	double meandepth;
	double silhouette_area;
	double similarity; // similarity to target
};

void CleanupExit()
{
	g_Context.Shutdown();
	exit (1);
}

geometry_msgs::Point vecToPt(XnVector3D pt) {
	geometry_msgs::Point ret;
	ret.x=pt.X/1000.0;
	ret.y=-pt.Y/1000.0;
	ret.z=pt.Z/1000.0;
	return ret;
}
geometry_msgs::Point32 vecToPt32(XnVector3D pt) {
	geometry_msgs::Point32 ret;
	ret.x=pt.X/1000.0;
	ret.y=-pt.Y/1000.0;
	ret.z=pt.Z/1000.0;
	return ret;
}


// A callback for an image
void imageCB(const sensor_msgs::ImageConstPtr& image_msg)
{
	// Convert the image from ROS format to OpenCV format
	cv_bridge::CvImagePtr cv_ptr;
	try	{
		cv_ptr = cv_bridge::toCvCopy(image_msg);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		return;
	}
	
	ROS_INFO_STREAM(boost::format("Callback got an image in format %s, size %dx%d")
		% cv_ptr->encoding % cv_ptr->image.rows % cv_ptr->image.cols );

	latest_rgb_ = cv_ptr->image.clone();
	got_rgb_ = true;
}

inline bool inbounds( const cv::Mat m, const cv::Point2i& p )
{
	return p.x > 0  &&  p.y > 0  &&  p.x < m.cols  &&  p.y < m.rows;
}

/*void getRGB(cv::Mat& rgb, cv::Mat& valid_mask)
{
	
	// Make a matrix the size of the RGB one
	cv::Size newsize(latest_rgb_.cols, latest_rgb_.rows);
	rgb.create(newsize, CV_8UC3);
	
	cv::Mat m(3, 3, CV_32F, cv::Scalar(0));
	m.at<float>(0,0) = 1;
	m.at<float>(1,1) = 1;
	m.at<float>(2,2) = 1;
	m.at<float>(0,2) = rgb_shift_h;
	m.at<float>(1,2) = rgb_shift_v;
	
	cv::warpPerspective(latest_rgb_, rgb, m, newsize);
//	cv::transform(latest_rgb_, shifted, m);
}*/

cv::Mat getRGB(const xn::ImageMetaData& imageMD)
{
	CV_Assert(imageMD.PixelFormat() == XN_PIXEL_FORMAT_RGB24);

	int rows = imageMD.YRes();
	int cols = imageMD.XRes();
	cv::Mat rgb(rows, cols, CV_8UC3);

	const XnRGB24Pixel* pRgbImage = imageMD.RGB24Data();
	memcpy( rgb.data, pRgbImage, cols*rows*3*sizeof(uchar) );

	cv::cvtColor( rgb, rgb, CV_RGB2BGR );
	
	cv::Size newsize(rgb.cols, rgb.rows);
	cv::Mat m(3, 3, CV_32F, cv::Scalar(0));
	m.at<float>(0,0) = 1;
	m.at<float>(1,1) = 1;
	m.at<float>(2,2) = rgb_scale_z;
	m.at<float>(0,2) = rgb_shift_h;
	m.at<float>(1,2) = rgb_shift_v;
	
	cv::Mat transformed(newsize, CV_8UC3);
	cv::warpPerspective(rgb, transformed, m, newsize);

	return transformed;
}

void getUserLabelImage(xn::SceneMetaData& sceneMD, cv::Mat& label_image)
{
	int rows = sceneMD.GetUnderlying()->pMap->Res.Y;
	int cols = sceneMD.GetUnderlying()->pMap->Res.X;
	
	// Data is 16-bit user labels
	cv::Mat tempmat(rows, cols, CV_16U);
	tempmat.data = (uchar*) sceneMD.GetUnderlying()->pData;
	
	// Convert to 8-bit (we never have more than 8-10 users anyway)
	tempmat.convertTo(label_image, CV_8U);
}


void getDepthImage(xn::DepthMetaData& depthMD, cv::Mat& depth_image)
{
	int rows = depthMD.GetUnderlying()->pMap->Res.Y;
	int cols = depthMD.GetUnderlying()->pMap->Res.X;
	
	// Data is 16-bit unsigned depths in mm
	cv::Mat tempmat(rows, cols, CV_16U);
	tempmat.data = (uchar*) depthMD.GetUnderlying()->pData;
	
	// Convert to floating point meters
	tempmat.convertTo(depth_image, CV_32F);
	depth_image /= 1000;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE
User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("[bk_skeletal_tracker] New User %d", nId);
	
	// TODO: See if this user was near a recently dropped UID.  If so, load that calibration
	
	// If we already calibrated on a user, just load that calibration
	if(g_bhasCal) {
		//g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(first_calibrated_user_, 0);
		ROS_INFO("[bk_skeletal_tracker] Loaded previous calibration of user %d", first_calibrated_user_);
	}
	// Detected first user: request calibration pose detection
	else
	{
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
	}
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE
User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	// TODO: Erase tracker
	ROS_INFO("[bk_skeletal_tracker] Lost user %d", nId);
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE
UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	ROS_INFO("[bk_skeletal_tracker] Pose (%s) detected for user %d", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	
	// If we already calibrated on a user, just load that calibration
	if(g_bhasCal) {
		//g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, 0);
		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(first_calibrated_user_, 0);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		ROS_INFO("[bk_skeletal_tracker] Loaded previous calibration of user %d", first_calibrated_user_);
	}
	// Detected pose of first user: start calibration
	else {
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
	}
}

// Callback: Started calibration
void XN_CALLBACK_TYPE
UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	ROS_INFO("[bk_skeletal_tracker] Calibration started for user %d", nId);
}

// Callback: Finished calibration
void XN_CALLBACK_TYPE
UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	if (bSuccess)
	{
		// Calibration succeeded - save this first calibration and start tracking the user
		ROS_INFO("[bk_skeletal_tracker] Calibration complete, now tracking user %d", nId);
		
		g_bhasCal=true;
		first_calibrated_user_ = nId;
		g_UserGenerator.GetSkeletonCap().SaveCalibrationData(nId, 0);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	
		// Save the user's cal
		
		// Get mask of this user
		xn::SceneMetaData sceneMD;
		cv::Mat label_image;
		g_UserGenerator.GetUserPixels(0, sceneMD);
		getUserLabelImage(sceneMD, label_image);
		label_image = (label_image == nId);
		
		xn::ImageMetaData imageMD;
		g_ImageGenerator.GetMetaData(imageMD);
		
		cv::Mat rgb, rgb_mask;
//		getRGB(rgb, rgb_mask);
		rgb = getRGB(imageMD);
		
		original_cal_.init(rgb, label_image);
		user_cal_ = original_cal_;
	}
	else
	{
		// Calibration failed
		ROS_INFO("[bk_skeletal_tracker] Calibration failed for user %d", nId);
		
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
	}
}


// this function is called each frame
void glutDisplay (void)
{
	static ros::Duration pub_interval(1.0/pub_rate_temp_);
	static ros::Time     last_pub(0.0);
	static int           num_skipped = 0;
	
	num_skipped++;
	
	ros::Time now_time = ros::Time::now();
	
	// Update stuff from OpenNI
//	g_Context.WaitAndUpdateAll();
	XnStatus status = g_Context.WaitAndUpdateAll();
	
	if( status != XN_STATUS_OK ){
		ROS_ERROR_STREAM("Updating context failed: " << status);
		return;
	}
	
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	xn::ImageMetaData imageMD;
	g_DepthGenerator.GetMetaData(depthMD);
	g_UserGenerator.GetUserPixels(0, sceneMD);
	g_ImageGenerator.GetMetaData(imageMD);
	
	cv::Mat depth_image;
	getDepthImage(depthMD, depth_image);
	
	double minval, maxval;
	cv::minMaxLoc(depth_image, &minval, &maxval);
	
	// Convert user pixels to an OpenCV image
	cv::Mat label_image;
	getUserLabelImage(sceneMD, label_image);
	
	sensor_msgs::PointCloud cloud;
	cloud.header.stamp    = now_time;
	cloud.header.frame_id = frame_id_;
	cloud.channels.resize(1);
	cloud.channels[0].name = "intensity";
	
	// Convert users into better format
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	g_UserGenerator.GetUsers(aUsers, nUsers);
	
	cv::Mat      this_mask;
	XnPoint3D    center_mass;
	double       pixel_area;
	cv::Scalar   s;
	vector<user> users;
		
	if( g_bhasCal && now_time-last_pub > pub_interval )
	{
		bool has_lock = false;
		last_pub = now_time;
		ROS_DEBUG_STREAM(num_skipped << " refreshes inbetween publishing");
		num_skipped = 0;
		
		cv::imshow( "Tracked user"        , user_cal_.getImage() );
		cv::imshow( "Original calibration", original_cal_.getImage() );
		
		cv::Mat rgb, rgb_mask;
//		getRGB(rgb, rgb_mask);
		rgb = getRGB(imageMD);
		
		for (unsigned int i = 0; i < nUsers; i++)
		{
			user this_user;
			this_user.uid = aUsers[i];
			
			// Bitwise mask of pixels belonging to this user
			this_mask = (label_image == this_user.uid);
			this_user.numpixels = cv::countNonZero(this_mask);
			
			// Compare this user to the target
			this_user.pc.init(rgb, this_mask);
			double similarity  = this_user.pc.compare(user_cal_    );
			double sim_to_orig = this_user.pc.compare(original_cal_);
			
			// Mean depth
			this_user.meandepth = cv::mean(depth_image, this_mask)[0];
		
			this_user.silhouette_area = 0;
		
			// Find the area of the silhouette in cartesian space
			for( int i=0; i<this_mask.rows; i++) {
				for( int j=0; j<this_mask.cols; j++ ) {
					if( this_mask.at<uchar>(i,j) != 0 )
					{
						pixel_area = cam_model_.getDeltaX(1, depth_image.at<float>(i,j))
										   * cam_model_.getDeltaY(1, depth_image.at<float>(i,j));
						this_user.silhouette_area += pixel_area;
					}
				}
			}
			// Find the center in 3D
			g_UserGenerator.GetCoM(this_user.uid, center_mass);
			this_user.center3d.point = vecToPt(center_mass);
		
			ROS_DEBUG_STREAM(boost::format("User %d: area %.3fm^2, mean depth %.3fm")
				% (unsigned int)this_user.uid % this_user.silhouette_area % this_user.meandepth);
		
			// Screen out unlikely users based on area
			if( this_user.meandepth > min_dist_ && this_user.silhouette_area < max_area_ && this_user.silhouette_area > min_area_ )
			{
				ROS_INFO_STREAM(boost::format("User %d   new: %.0f --- orig: %.0f")
					% ((int)this_user.uid) % (100*similarity) % (100*sim_to_orig) );
					
				if( similarity > PersonCal::getMatchThresh() ) {
					user_cal_.update(rgb, this_mask);
				}
				else{
					if( sim_to_orig > PersonCal::getMatchThresh() ) {
						ROS_WARN_STREAM("Reset to original calibration");
						user_cal_ = original_cal_;
					}
				}
			
				std::stringstream window_name;
				window_name << "user_" << ((int)this_user.uid);
				cv::imshow(window_name.str(), this_user.pc.getImage());
			
			
				ROS_DEBUG("Accepted user");
				users.push_back(this_user);
				
				// Visualization
				geometry_msgs::Point32 p;
				p.x = this_user.center3d.point.x;
				p.y = this_user.center3d.point.y;
				p.z = this_user.center3d.point.z;
				cloud.points.push_back(p);
				cloud.channels[0].values.push_back(0.0f);
			}
		}
	
	
		// Try to associate the tracker with a user
		if( latest_tracker_.first != "" )
		{
			// Transform the tracker to this time. Note that the pos time is updated but not the restamp.
			tf::Point pt;
			tf::pointMsgToTF(latest_tracker_.second.pos.pos, pt);
			tf::Stamped<tf::Point> loc(pt, latest_tracker_.second.pos.header.stamp, latest_tracker_.second.pos.header.frame_id);
			try {
				tfl_->transformPoint(frame_id_, now_time-ros::Duration(.1), loc, latest_tracker_.second.pos.header.frame_id, loc);
				latest_tracker_.second.pos.header.stamp    = now_time;
				latest_tracker_.second.pos.header.frame_id = frame_id_;
				latest_tracker_.second.pos.pos.x = loc[0];
				latest_tracker_.second.pos.pos.y = loc[1];
				latest_tracker_.second.pos.pos.z = loc[2];
			}
			catch (tf::TransformException& ex) {
				ROS_ERROR("(finding) Could not transform person to this time");
			}
			  
			people_msgs::PositionMeasurement pos;
			if( users.size() > 0 )
			{
				std::stringstream users_ss;
				users_ss << boost::format("(finding) Tracker \"%s\" = (%.2f,%.2f) Users = ") 
					% latest_tracker_.first % latest_tracker_.second.pos.pos.x   % latest_tracker_.second.pos.pos.y;
			
				// Find the closest user to the tracker
				user closest;
				closest.distance = BIGDIST_M;
	
				foreach(user u, users)
				{
					u.distance = pow(latest_tracker_.second.pos.pos.x - u.center3d.point.x, 2.0)
						         + pow(latest_tracker_.second.pos.pos.y - u.center3d.point.y, 2.0);
					
					users_ss << boost::format("(%.2f,%.2f), ")
				                % u.center3d.point.x % u.center3d.point.y;
					
					if( u.distance < closest.distance ) {	closest = u; }
				}
				
				string users_s = users_ss.str();
				ROS_DEBUG_STREAM(users_s);
			
			
				if( closest.distance < association_dist_  )
				{
					// Convert to a PositionMeasurement message
					pos.header.stamp    = now_time;
					pos.header.frame_id = frame_id_;
					pos.name            = "openni";
					pos.object_id       = latest_tracker_.second.pos.object_id;
					pos.pos.x           = closest.center3d.point.x;
					pos.pos.y           = closest.center3d.point.y;
					pos.pos.z           = closest.center3d.point.z;
				  pos.reliability     = reliability_;
				  pos.initialization  = 0;
			
				  pos.covariance[0] = variance_xy_; pos.covariance[1] = 0.0;          pos.covariance[2] = 0.0;
				  pos.covariance[3] = 0.0;          pos.covariance[4] = variance_xy_; pos.covariance[5] = 0.0;
				  pos.covariance[6] = 0.0;          pos.covariance[7] = 0.0;          pos.covariance[8] = 0.40;
				  
				  pos_pub_.publish(pos);
				  has_lock = true;
				  ROS_DEBUG_STREAM(boost::format("(finding) Published measurement for person \"%s\" at (%.2f,%.2f,%.2f) (user %d) distance %.2f")
				  	%pos.object_id %pos.pos.x %pos.pos.y %pos.pos.z %closest.uid %closest.distance );
				  
					// Visualization
					geometry_msgs::Point32 p;
					p.x = pos.pos.x;
					p.y = pos.pos.y;
					p.z = pos.pos.z;
					cloud.points.push_back(p);
					cloud.channels[0].values.push_back(1.0f);
				}
				else
					ROS_DEBUG_STREAM(boost::format("(finding) No association (distance %.2f) ")
						%closest.distance );
			}
			else
				ROS_DEBUG("(finding) No users");
		}
		else
			ROS_DEBUG("(finding) No tracker");
	
		// Visualization
		cloud_pub_.publish(cloud);
		
		ROS_DEBUG_STREAM("(finding) lock = " << (has_lock ? "true" : "false" ) );
		std_msgs::Float64 b;
		b.data = (has_lock ? 1.0 : 0.0 );
		has_lock_pub_.publish(b);
	}
	
	cv::Mat rgb, rgb_mask;
//	getRGB(rgb, rgb_mask);
	rgb = getRGB(imageMD);
	cv::imshow("rgb", rgb);
	cv::waitKey(5);
	
	// Draw OpenGL display
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
	glDisable(GL_TEXTURE_2D);
	DrawDepthMap(depthMD, sceneMD);
	glutSwapBuffers();
	
	ros::Duration actual_loop_time = ros::Time::now() - now_time;
	ROS_DEBUG_STREAM(boost::format("[bk_skeletal_tracker] Loop took %f seconds.") % (actual_loop_time.toSec()) );
	
	// Allow callbacks to occur
	ros::spinOnce();
}

void glutIdle (void)
{
	if (!ros::ok()) {
		CleanupExit();
	}
	
	// Display the frame
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:
			CleanupExit();
		case 'b':
			// Draw background?
			g_bDrawBackground = !g_bDrawBackground;
			break;
		case 'x':
			// Draw pixels at all?
			g_bDrawPixels = !g_bDrawPixels;
			break;
		case 's':
			// Draw Skeleton?
			g_bDrawSkeleton = !g_bDrawSkeleton;
			break;
		case 'i':
			// Print label?
			g_bPrintID = !g_bPrintID;
			break;
		case 'l':
			// Print ID & state as label, or only ID?
			g_bPrintState = !g_bPrintState;
			break;
		case'p':
			g_bPause = !g_bPause;
			break;
	}
}

void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("User Tracking");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);
	
	//glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}

#define CHECK_RC(nRetVal, what)   \
if (nRetVal != XN_STATUS_OK)      \
{                                 \
  ROS_ERROR("[bk_skeletal_tracker] %s failed: %s", what, xnGetStatusString(nRetVal));\
  return nRetVal;                 \
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bk_skeletal_tracker");
	ros::NodeHandle nh_;
	ros::NodeHandle pnh("~");
	tf::TransformListener tfl;
	tfl_ = &tfl;
	image_transport::ImageTransport it(nh_);
	
	frame_id_ = "derpderpderp";
	pnh.getParam("camera_frame_id", frame_id_);
	
	pnh.param("smoothing_factor", smoothing_factor_, 0.5);
	pnh.param("reliability"     , reliability_     , 0.5);
	pnh.param("pub_rate"        , pub_rate_temp_   , 5.0);
	pnh.param("min_dist"        , min_dist_        , 0.1);
	pnh.param("association_dist", association_dist_, 1.0);
	pnh.param("min_area"        , min_area_        , 0.0);
	pnh.param("max_area"        , max_area_        , 1.0);
	pnh.param("variance_xy"     , variance_xy_     , 0.3);
	
	double tempdouble;
	pnh.param("match_threshold" , tempdouble       , 0.9);
	PersonCal::setMatchThresh(tempdouble);
	
	int hbins, sbins;
	pnh.param("hist_h_bins" , hbins       , 30);
	pnh.param("hist_s_bins" , sbins       , 30);
	PersonCal::setHistogramParameters(hbins, sbins);
	
	pnh.param("rgb_shift_v", rgb_shift_v , 0.0);
	pnh.param("rgb_shift_h", rgb_shift_h , 0.0);
	pnh.param("rgb_scale_z", rgb_scale_z , 1.0);
	
	
	ROS_INFO_STREAM(boost::format("[bk_skeletal_tracker] Pub rate        = %f"    ) % pub_rate_temp_);
	ROS_INFO_STREAM(boost::format("[bk_skeletal_tracker] Frame_id        = \"%s\"") % frame_id_);
	ROS_INFO_STREAM(boost::format("[bk_skeletal_tracker] Smoothing factor= %.2f"  ) % smoothing_factor_);
	ROS_INFO_STREAM(boost::format("[bk_skeletal_tracker] Min distance    = %.2f"  ) % min_dist_);
	ROS_INFO_STREAM(boost::format("[bk_skeletal_tracker] Ass. distance   = %.2f"  ) % association_dist_);
	ROS_INFO_STREAM(boost::format("[bk_skeletal_tracker] Area bounds     = [%.2f,%.2f]" ) % min_area_ %max_area_);
	ROS_INFO_STREAM(boost::format("[bk_skeletal_tracker] x, y variance   = %.2f"  ) % variance_xy_);
	ROS_INFO_STREAM("[bk_skeletal_tracker] Match threshold = " << PersonCal::getMatchThresh());
	ROS_INFO_STREAM("[bk_skeletal_tracker] RGB shift h=" << rgb_shift_h << ", v=" << rgb_shift_v << ", scale=" << rgb_scale_z);
	
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("bk_skeletal_tracker/people_cloud",0);
	
	// Advertise a position measure message.
	pos_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("/people_tracker_measurements",1);
  
  has_lock_pub_ = nh_.advertise<std_msgs::Float64>("bk_skeletal_tracker/has_lock", 0);
  
	// Subscribe to people tracker filter state
	latest_tracker_.first = "";
	ros::Subscriber pos_sub = nh_.subscribe("people_tracker_filter", 5, &posCallback);
	
	// Subscribe to RGB
//	image_transport::Subscriber image_sub = it.subscribe("in_image", 1, boost::bind(&imageCB, _1) );
	
	// Subscribe to camera info
	got_cam_info_ = false;
	ros::Subscriber cam_info_sub = nh_.subscribe("camera/rgb/camera_info", 1, cam_info_cb);
	ROS_INFO("[bk_skeletal_tracker] Waiting for camera info...");
	
	// Get one message and then unsubscribe
	while( !got_cam_info_ ){ //|| !got_rgb_ ) {
		ros::spinOnce();
	}
	cam_info_sub.shutdown();
	
	
	XnStatus nRetVal = XN_STATUS_OK;
	
	// Initialize OpenNI with a saved configuration
	std::string configFilename = ros::package::getPath("bk_skeletal_tracker") + "/bk_skeletal_tracker.xml";
	nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
	CHECK_RC(nRetVal, "InitFromXml");
	
	// The configuration should have created a depth generator node
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");
	
	
	// The configuration should have created an image generator node
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_ImageGenerator.Create(g_Context);
		ROS_INFO("Creating image generator");
		CHECK_RC(nRetVal, "Find image generator");
	} else {
		ROS_INFO("Found image generator");
	}
	g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
	
	
	// See if a user generator node exists.  If not, create one.
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}
	
	// Make sure that the user generator supports skeleton capture
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		ROS_INFO("[bk_skeletal_tracker] Supplied user generator doesn't support skeleton");
		return 1;
	}
	
	bool ret;
	ret = g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT);
	ROS_INFO_STREAM("User generator alt viewpoint: " << (ret?"true":"false") );
	ret = g_DepthGenerator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT);
	ROS_INFO_STREAM("Depth generator alt viewpoint: " << (ret?"true":"false") );
	ret = g_ImageGenerator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT);
	ROS_INFO_STREAM("Image generator alt viewpoint: " << (ret?"true":"false") );
	
	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
	
	// Register callbacks to occur on user state change
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	
	// Register callbacks to occur on calibration change
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);
	
	// If the user generator requires a calibration pose, set up some more callbacks.
	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = true;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			ROS_INFO("[bk_skeletal_tracker] Pose required, but not supported");
			return 1;
		}
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
		ROS_INFO("[bk_skeletal_tracker] User generator requires calibration pose (%s)", g_strPose);
	}
	else {
		ROS_INFO("[bk_skeletal_tracker] No calibration pose required");
	}
	
	// Set up the skeleton generator
	g_UserGenerator.GetSkeletonCap().SetSmoothing(smoothing_factor_);
	
	//g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);
	
	// Kick things off
	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
	
	glInit(&argc, argv);
	glutMainLoop();
}
