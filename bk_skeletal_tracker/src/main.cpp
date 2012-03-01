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
#include <ros/package.h>

#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <body_msgs/Skeletons.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

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

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

// Is a pose required for calibration? If so, g_strPose holds its name.
XnBool g_bNeedPose   = false;
XnChar g_strPose[20] = "";

// Have we successfully calibrated a user? If so, which user?
XnBool      g_bhasCal = false;
XnUserID    first_calibrated_user_;

// Controls what is drawn to the screen
XnBool g_bDrawBackground = true;
XnBool g_bDrawPixels     = true;
XnBool g_bDrawSkeleton   = true;
XnBool g_bPrintID        = true;
XnBool g_bPrintState     = true;
XnBool g_bPause          = false;

// ROS stuff
ros::Publisher pmap_pub,skel_pub;
std::string    frame_id_;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
void CleanupExit()
{
	g_Context.Shutdown();
	exit (1);
}

geometry_msgs::Point vecToPt(XnVector3D pt){
   geometry_msgs::Point ret;
   ret.x=pt.X/1000.0;
   ret.y=-pt.Y/1000.0;
   ret.z=pt.Z/1000.0;
   return ret;
}
geometry_msgs::Point32 vecToPt3(XnVector3D pt){
   geometry_msgs::Point32 ret;
   ret.x=pt.X/1000.0;
   ret.y=-pt.Y/1000.0;
   ret.z=pt.Z/1000.0;
   return ret;
}

void getSkeletonJoint(XnUserID player, body_msgs::SkeletonJoint &j, XnSkeletonJoint name){
   XnSkeletonJointPosition joint1;
   g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, name, joint1);
   j.position= vecToPt(joint1.position);
   j.confidence = joint1.fConfidence;
}

void getSkeleton(XnUserID player,body_msgs::Skeleton &skel)
{
   skel.playerid=player;
   getSkeletonJoint(player,skel.head,XN_SKEL_HEAD);
   getSkeletonJoint(player,skel.neck,XN_SKEL_NECK);
   getSkeletonJoint(player,skel.left_shoulder,XN_SKEL_LEFT_SHOULDER);
   getSkeletonJoint(player,skel.left_elbow,XN_SKEL_LEFT_ELBOW);
   getSkeletonJoint(player,skel.left_hand,XN_SKEL_LEFT_HAND);
   getSkeletonJoint(player,skel.right_shoulder,XN_SKEL_RIGHT_SHOULDER);
   getSkeletonJoint(player,skel.right_elbow,XN_SKEL_RIGHT_ELBOW);
   getSkeletonJoint(player,skel.right_hand,XN_SKEL_RIGHT_HAND);
   getSkeletonJoint(player,skel.torso,XN_SKEL_TORSO);
   getSkeletonJoint(player,skel.left_hip,XN_SKEL_LEFT_HIP);
   getSkeletonJoint(player,skel.left_knee,XN_SKEL_LEFT_KNEE);
   getSkeletonJoint(player,skel.left_foot,XN_SKEL_LEFT_FOOT);
   getSkeletonJoint(player,skel.right_hip,XN_SKEL_RIGHT_HIP);
   getSkeletonJoint(player,skel.right_knee,XN_SKEL_RIGHT_KNEE);
   getSkeletonJoint(player,skel.right_foot,XN_SKEL_RIGHT_FOOT);
}

void getPolygon(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2, mapping_msgs::PolygonalMap &pmap)
{
   XnSkeletonJointPosition joint1, joint2;
   g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
   g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);
   if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
   {
      return;
   }
   geometry_msgs::Polygon p;
   p.points.push_back(vecToPt3(joint1.position));
   p.points.push_back(vecToPt3(joint2.position));
   pmap.polygons.push_back(p);
}

void ptdist(geometry_msgs::Polygon p)
{
	geometry_msgs::Point32 p1=p.points.back(),p2=p.points.front();
	ROS_INFO("[bk_skeletal_tracker] Shoulder dist %.02f ", sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z)));
}

void getSkels(std::vector<mapping_msgs::PolygonalMap> &pmaps, body_msgs::Skeletons &skels)
{
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	g_UserGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			body_msgs::Skeleton skel;
			getSkeleton(aUsers[i],skel);
			skels.skeletons.push_back(skel);

			mapping_msgs::PolygonalMap pmap;
			getPolygon(aUsers[i], XN_SKEL_HEAD          , XN_SKEL_NECK          , pmap);
			getPolygon(aUsers[i], XN_SKEL_NECK          , XN_SKEL_LEFT_SHOULDER , pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER , XN_SKEL_LEFT_ELBOW    , pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER , XN_SKEL_RIGHT_SHOULDER, pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_ELBOW    , XN_SKEL_LEFT_HAND     , pmap);
			getPolygon(aUsers[i], XN_SKEL_NECK          , XN_SKEL_RIGHT_SHOULDER, pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW   , pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_ELBOW   , XN_SKEL_RIGHT_HAND    , pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER , XN_SKEL_TORSO         , pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO         , pmap);
			getPolygon(aUsers[i], XN_SKEL_TORSO         , XN_SKEL_LEFT_HIP      , pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_HIP      , XN_SKEL_LEFT_KNEE     , pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_KNEE     , XN_SKEL_LEFT_FOOT     , pmap);
			getPolygon(aUsers[i], XN_SKEL_TORSO         , XN_SKEL_RIGHT_HIP     , pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_HIP     , XN_SKEL_RIGHT_KNEE    , pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_KNEE    , XN_SKEL_RIGHT_FOOT    , pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_HIP      , XN_SKEL_RIGHT_HIP     , pmap);
			
			pmaps.push_back(pmap);
		}
	}
}


// Callback: New user was detected
void XN_CALLBACK_TYPE
User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("[bk_skeletal_tracker] New User %d", nId);

	// If we already calibrated on a user, just load that calibration
	if(false){//g_bhasCal){
		//g_UserGenerator.GetSkeletonCap().LoadCalibrationData(first_calibrated_user_, 0);
		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, 0);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
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
	ROS_INFO("[bk_skeletal_tracker] Lost user %d", nId);
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE
UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	ROS_INFO("[bk_skeletal_tracker] Pose (%s) detected for user %d", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	
	// If we already calibrated on a user, just load that calibration
	if(false){//g_bhasCal){
		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, 0);
		//g_UserGenerator.GetSkeletonCap().LoadCalibrationData(first_calibrated_user_, 0);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		ROS_INFO("[bk_skeletal_tracker] Loaded previous calibration of user %d", first_calibrated_user_);
	}
	// Detected pose of first user: start calibration
	else{
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
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);
	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);

	glDisable(GL_TEXTURE_2D);

	if (!g_bPause) {
		// Read next available data
		g_Context.WaitAndUpdateAll();
	}
	
	ros::Time tstamp=ros::Time::now();
	
	// Update the OpenGL display
	g_DepthGenerator.GetMetaData(depthMD);
	g_UserGenerator.GetUserPixels(0, sceneMD);
	DrawDepthMap(depthMD, sceneMD);
	
	
	// Publish some ROS stuff
	std::vector<mapping_msgs::PolygonalMap> pmaps;
	body_msgs::Skeletons skels;
	getSkels(pmaps,skels);

	ROS_INFO_THROTTLE(5,"[bk_skeletal_tracker] Skels size %d ",pmaps.size());
	if(pmaps.size())
	{
		skels.header.stamp    = tstamp;
		skels.header.seq      = depthMD.FrameID();
		skels.header.frame_id = frame_id_;
		skel_pub.publish(skels);
		
		pmaps.front().header.stamp    = tstamp;
		pmaps.front().header.seq      = depthMD.FrameID();
		pmaps.front().header.frame_id = frame_id_;
		pmap_pub.publish(pmaps[0]);
	}
	
	//TODO: Weed out and stop tracking users with low confidence, and inhibit them for a bit
	
	glutSwapBuffers();

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

	frame_id_ = "derpderpderp";//("camera_depth_frame");
	pnh.getParam("camera_frame_id", frame_id_);
	ROS_INFO("[bk_skeletal_tracker] Frame_id = \"%s\"", frame_id_.c_str());
	
	pmap_pub = nh_.advertise<mapping_msgs::PolygonalMap> ("skeletonpmaps", 1);
	skel_pub = nh_.advertise<body_msgs::Skeletons>       ("skeletons", 1);

	XnStatus nRetVal = XN_STATUS_OK;

	// Initialize OpenNI with a saved configuration
	std::string configFilename = ros::package::getPath("bk_skeletal_tracker") + "/bk_skeletal_tracker.xml";
	nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
	CHECK_RC(nRetVal, "InitFromXml");

	// The configuration should have created a depth generator node
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");

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
	else{
		ROS_INFO("[bk_skeletal_tracker] No calibration pose required");
	}

	// Set up the skeleton generator
	//g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);

	// Kick things off
	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	glInit(&argc, argv);
	glutMainLoop();
}
