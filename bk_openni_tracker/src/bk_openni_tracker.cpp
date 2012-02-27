// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <body_msgs/Skeletons.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using std::string;
static const char WINDOW[] = "Detected Users";

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

ros::Publisher pmap_pub, skel_pub;
image_transport::Publisher image_pub;

void getUserLabelImage(const xn::SceneMetaData& sceneMD, cv::Mat& label_image)
{
	int rows = sceneMD.GetUnderlying()->pMap->Res.Y;
	int cols = sceneMD.GetUnderlying()->pMap->Res.X;

	// std::cout << "Rows: " << rows << " Cols: " << cols << std::endl;
	cv::Mat tempmat(rows, cols, CV_16U);
	tempmat.data = (uchar*) sceneMD.GetUnderlying()->pData;
	cv::Mat tempmat2 = tempmat.clone();
	tempmat2.convertTo(label_image, CV_8U);
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

void getSkeleton(XnUserID player,body_msgs::Skeleton &skel){
   skel.playerid=player;
   getSkeletonJoint(player, skel.head          ,XN_SKEL_HEAD);
   getSkeletonJoint(player, skel.neck          ,XN_SKEL_NECK);
   getSkeletonJoint(player, skel.left_shoulder ,XN_SKEL_LEFT_SHOULDER);
   getSkeletonJoint(player, skel.left_elbow    ,XN_SKEL_LEFT_ELBOW);
   getSkeletonJoint(player, skel.left_hand     ,XN_SKEL_LEFT_HAND);
   getSkeletonJoint(player, skel.right_shoulder,XN_SKEL_RIGHT_SHOULDER);
   getSkeletonJoint(player, skel.right_elbow   ,XN_SKEL_RIGHT_ELBOW);
   getSkeletonJoint(player, skel.right_hand    ,XN_SKEL_RIGHT_HAND);
   getSkeletonJoint(player, skel.torso         ,XN_SKEL_TORSO);
   getSkeletonJoint(player, skel.left_hip      ,XN_SKEL_LEFT_HIP);
   getSkeletonJoint(player, skel.left_knee     ,XN_SKEL_LEFT_KNEE);
   getSkeletonJoint(player, skel.left_foot     ,XN_SKEL_LEFT_FOOT);
   getSkeletonJoint(player, skel.right_hip     ,XN_SKEL_RIGHT_HIP);
   getSkeletonJoint(player, skel.right_knee    ,XN_SKEL_RIGHT_KNEE);
   getSkeletonJoint(player, skel.right_foot    ,XN_SKEL_RIGHT_FOOT);
}

void getPolygon(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2, mapping_msgs::PolygonalMap &pmap){
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

void getSkels(std::vector<mapping_msgs::PolygonalMap> &pmaps, body_msgs::Skeletons &skels){
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	skels.skeletons.clear();
	g_UserGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			body_msgs::Skeleton skel;
			getSkeleton(aUsers[i],skel);
			skels.skeletons.push_back(skel);

			mapping_msgs::PolygonalMap pmap;
			getPolygon(aUsers[i], XN_SKEL_HEAD          , XN_SKEL_NECK, pmap);
			// printPt(aUsers[i], XN_SKEL_RIGHT_HAND);
			getPolygon(aUsers[i], XN_SKEL_NECK          , XN_SKEL_LEFT_SHOULDER, pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER , XN_SKEL_LEFT_ELBOW, pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER , XN_SKEL_RIGHT_SHOULDER, pmap);
			// ptdist(pmap.polygons.back());
			getPolygon(aUsers[i], XN_SKEL_LEFT_ELBOW    , XN_SKEL_LEFT_HAND, pmap);
			getPolygon(aUsers[i], XN_SKEL_NECK          , XN_SKEL_RIGHT_SHOULDER, pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW, pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_ELBOW   , XN_SKEL_RIGHT_HAND, pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER , XN_SKEL_TORSO, pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO, pmap);
			getPolygon(aUsers[i], XN_SKEL_TORSO         , XN_SKEL_LEFT_HIP, pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_HIP      , XN_SKEL_LEFT_KNEE, pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_KNEE     , XN_SKEL_LEFT_FOOT, pmap);
			getPolygon(aUsers[i], XN_SKEL_TORSO         , XN_SKEL_RIGHT_HIP, pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_HIP     , XN_SKEL_RIGHT_KNEE, pmap);
			getPolygon(aUsers[i], XN_SKEL_RIGHT_KNEE    , XN_SKEL_RIGHT_FOOT, pmap);
			getPolygon(aUsers[i], XN_SKEL_LEFT_HIP      , XN_SKEL_RIGHT_HIP, pmap);
			// getSkel(aUsers[i],pmap);
			pmaps.push_back(pmap);
		}
	}
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, now tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;

        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int main(int argc, char **argv) {
	ros::init(argc, argv, "openni_tracker");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	//cv::namedWindow(WINDOW);

	skel_pub = nh.advertise<body_msgs::Skeletons> ("skeletons", 1);
	pmap_pub = nh.advertise<mapping_msgs::PolygonalMap> ("skeletonpmaps", 1);
	image_pub = it.advertise("silhouettes", 1);

	string configFilename = ros::package::getPath("bk_openni_tracker") + "/bk_openni_tracker.xml";
	XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
	CHECK_RC(nRetVal, "InitFromXml");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_WARN("Supplied user generator doesn't support skeleton");
		return 1;
	}

	XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}
		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(10);
	
	ros::NodeHandle pnh("~");
	string frame_id("derpderpderp");//("camera_depth_frame");
	pnh.getParam("camera_frame_id", frame_id);
	ROS_INFO("[bk_openni_tracker] Using frame_id = \"%s\"", frame_id.c_str());

	std::vector<mapping_msgs::PolygonalMap> pmaps;
	body_msgs::Skeletons skels;
	ros::Time tstamp;
	xn::DepthMetaData depthMD;
	xn::SceneMetaData sceneMD;
	
	cv::Mat user_label_image;
	unsigned int img_seq = 0;
	cv_bridge::CvImage cv_img;
	sensor_msgs::Image ros_image;

	while (ros::ok())
	{
		g_Context.WaitAndUpdateAll();
		
		publishTransforms(frame_id);

		pmaps.clear();
		skels.skeletons.clear();
		tstamp=ros::Time::now();

		// Get image with user labels
		g_UserGenerator.GetUserPixels(0, sceneMD);
		getUserLabelImage(sceneMD, user_label_image);
		//cv::imshow(WINDOW, user_label_image*70 );
		//cv::waitKey(3);
		
		// Convert label image to ROS message and publish 
		cv_img.image = user_label_image;
		cv_img.encoding = sensor_msgs::image_encodings::MONO8;
		cv_img.header.seq = ++img_seq;
		cv_img.header.stamp = tstamp;
		cv_img.toImageMsg(ros_image);
		image_pub.publish(ros_image);
		
		// Get current skeleton objects and polygonal maps representing skeletons
		getSkels(pmaps,skels);
		ROS_DEBUG("skels size %d \n",pmaps.size());
		if(pmaps.size())
		{
			// Fill in and publish skeletons object
			g_DepthGenerator.GetMetaData(depthMD);
			skels.header.stamp    = tstamp;
			skels.header.seq      = depthMD.FrameID();
			skels.header.frame_id = "/camera_depth_optical_frame";
			skel_pub.publish(skels);

			pmaps.front().header.stamp    = tstamp;
			pmaps.front().header.seq      = depthMD.FrameID();
			pmaps.front().header.frame_id = "/camera_depth_optical_frame";
			pmap_pub.publish(pmaps[0]);
		}

		r.sleep();
	}

	//cv::destroyWindow(WINDOW);
	g_Context.Shutdown();
	return 0;
}
