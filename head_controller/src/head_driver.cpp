#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <phidget21.h>
#include <boost/format.hpp>

const double pi = 3.1415926;
double desired_pan_ = 0.0;
double actual_pan_ = 0.0;

using geometry_msgs::Twist;
using geometry_msgs::Vector3;

double pan_ang_min_ = 0.0, pan_ang_max_ = 0.0, pan_ang_center_ = 0.0;

double toServoFrame(double angle)
{
	angle += pan_ang_center_;
	
	if( angle < pan_ang_min_ ) {
		angle = pan_ang_min_;
	}
	
	if( angle > pan_ang_max_ ) {
		angle = pan_ang_max_;
	}
	
	return pan_ang_max_-angle;
}

double fromServoFrame(double angle)
{
	return (pan_ang_max_ - angle) - pan_ang_center_;
}

void panAngleCallback(const std_msgs::Float64& msg)
{
	// Low pass filter - avoid jolts
	static double lpf   = 0.0;
	const  double alpha = 0.7;
	lpf = (lpf)*(1.0-alpha) + (msg.data)*(alpha);
	
	// Convert to degrees
	desired_pan_ = (lpf*180.0/pi);
}

// Called when a Phidget is attached
int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName  (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	ROS_INFO("[head_driver] %s %10d attached!", name, serialNo);

	return 0;
}

// Called when a Phidget is detached
int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName  (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	ROS_INFO("[head_driver] %s %10d detached!", name, serialNo);

	return 0;
}

// Called when an error occurs in a Phidget
int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{
	ROS_ERROR("[head_driver] Error handled. %d - %s", ErrorCode, Description);
	return 0;
}

// Called whenever the servo changes position
int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
	actual_pan_ = fromServoFrame(Value);
	//ROS_INFO_THROTTLE(2,"[head_driver] Callback: Current Position: %.2f (%.2f in servo frame)", fromServoFrame(Value) , Value);
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
int display_properties(CPhidgetAdvancedServoHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType   ((CPhidgetHandle)phid, &ptr     );
	CPhidget_getSerialNumber ((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version );

	CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);

	ROS_INFO("[head_driver] %s, Serial: %10d, Version: %8d, # Motors: %d", ptr, serialNo, version, numMotors);

	return 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "head_driver");
	ros::NodeHandle nh("~");
	tf::TransformBroadcaster br;
	
	// Publish the state of the servo for diagnostics
	ros::Publisher head_des_pos_pub = nh.advertise<Twist>("head_des_pos"  , 1);
	ros::Publisher head_pos_pub     = nh.advertise<Twist>("head_pos"  , 1);
	ros::Publisher head_error_pub   = nh.advertise<Twist>("head_error", 1);
	ros::Publisher head_speed_pub   = nh.advertise<Twist>("head_speed", 1);
	ros::Publisher head_accel_pub   = nh.advertise<Twist>("head_accel", 1);
	ros::Publisher head_current_pub = nh.advertise<std_msgs::Float64>("head_current", 1);
	
	// We control a servo that determines the link between parent_frame and child_frame
	std::string parent_frame, child_frame;
	nh.param<std::string>("parent_tf_frame", parent_frame , "NULL");
	nh.param<std::string>("child_tf_frame" , child_frame  , "NULL");
	ROS_INFO("[head_driver] Broadcasting TF from \"%s\" to \"%s\"",	parent_frame.c_str(), child_frame.c_str());
	
	double feedforward_kv;
	
	// Constraints on pan velocity and acceleration
	double pan_vel_max, pan_acc_max;
	nh.param("pan_angle_center", pan_ang_center_,  90.0);
	nh.param("pan_angle_min"   , pan_ang_min_   ,   0.0);
	nh.param("pan_angle_max"   , pan_ang_max_   , 180.0);
	nh.param("pan_vel_max"     , pan_vel_max    ,   0.0);
	nh.param("pan_acc_max"     , pan_acc_max    ,   0.0);
	nh.param("feedforward_kv"  , feedforward_kv ,   0.0);
	ROS_INFO("[head_driver] Accel: %.2f, Velocity: %.2f", pan_acc_max, pan_vel_max);
	ROS_INFO("[head_driver] Angle min: %.2f center: %.2f max: %.2f", pan_ang_min_, pan_ang_center_, pan_ang_max_);
	ROS_INFO_STREAM(boost::format("[head_driver] Feedforward KV = %.3f") %feedforward_kv);
	
	// Control loop rate
	double loop_rate_dbl;
	nh.param("loop_rate", loop_rate_dbl, 10.0);
	ROS_INFO("[head_driver] Loop rate is %.2fHz", loop_rate_dbl);
	ros::Rate loop_rate = ros::Rate(loop_rate_dbl);
	
	//Declare an advanced servo handle and create the advanced servo object
	CPhidgetAdvancedServoHandle servo = 0;
	CPhidgetAdvancedServo_create(&servo);
	
	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
	CPhidget_set_OnError_Handler ((CPhidgetHandle)servo, ErrorHandler , NULL);
	
	//open the device for connections
	CPhidget_open((CPhidgetHandle)servo, -1);

	int result;
	const char *err;
	// Wait for the servo to be attached
	ROS_INFO("[head_driver] Waiting for Phidget to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		ROS_INFO("[head_driver] Problem waiting for attachment: %s\n", err);
		return 0;
	}
	
	//Display the properties of the attached device
	display_properties(servo);
	
	CPhidgetAdvancedServo_setServoType(servo, 0, PHIDGET_SERVO_HITEC_HS485HB);
	
	//Set up the servo's velocity and acceleration limits
	double servo_min_accel, servo_max_accel;
	CPhidgetAdvancedServo_getAccelerationMin(servo, 0, &servo_min_accel);
	CPhidgetAdvancedServo_getAccelerationMax(servo, 0, &servo_max_accel);
	ROS_INFO_STREAM(boost::format("[head_driver] Servo accel limits: %.2f < x < %.2f") % servo_min_accel % servo_max_accel);
	
	if( pan_acc_max > servo_max_accel ){
		ROS_WARN("[head_driver] Servo accel to high, clamping to acceptable value.");
		pan_acc_max = servo_max_accel;
	}
	else if( pan_acc_max < servo_min_accel ){
		ROS_WARN("[head_driver] Servo accel to low, clamping to acceptable value.");
		pan_acc_max = servo_min_accel;
	}
	
	CPhidgetAdvancedServo_setAcceleration  (servo, 0, pan_acc_max);
	CPhidgetAdvancedServo_setVelocityLimit (servo, 0, pan_vel_max);
	ROS_INFO("[head_driver] Accel: %.2f, Velocity: %.2f", pan_acc_max, pan_vel_max);
	
	CPhidgetAdvancedServo_setServoType(servo, 0, PHIDGET_SERVO_HITEC_HS485HB);
	
	// Center the servo, engage the drive.
	CPhidgetAdvancedServo_setSpeedRampingOn(servo, 0, 1);
	
	CPhidgetAdvancedServo_setEngaged (servo, 0, 1);
	CPhidgetAdvancedServo_setPosition(servo, 0, toServoFrame(0.0));

	ros::Subscriber angle_sub_ = nh.subscribe("/pan_command", 1, &panAngleCallback);
	ros::Duration(0.5).sleep();
	
	double curr_pos, curr_vel, curr_acc, curr_curr;
	tf::Transform transform;
	while( ros::ok() )
	{
		//Get current motor position, publish the current kinematics for diagnostics
		if(CPhidgetAdvancedServo_getPosition(servo, 0, &curr_pos) == EPHIDGET_OK)
		{
			CPhidgetAdvancedServo_getVelocity    (servo, 0, &curr_vel);
			CPhidgetAdvancedServo_getAcceleration(servo, 0, &curr_acc);
			CPhidgetAdvancedServo_getCurrent     (servo, 0, &curr_curr);
			
			std_msgs::Header h;
			h.stamp    = ros::Time::now();
			h.frame_id = "pan_link";
			
			Vector3 zero;
			zero.x=0; zero.y=0; zero.z=0;
			Twist twist;
			twist.linear = zero; twist.angular = zero;
			
			twist.angular.z = desired_pan_;
			head_des_pos_pub.publish(twist);
			
			twist.angular.z = fromServoFrame(curr_pos);
			head_pos_pub.publish(twist);
			
			twist.angular.z = curr_vel;
			head_speed_pub.publish(twist);
			
			twist.angular.z = curr_acc;
			head_accel_pub.publish(twist);
			
			twist.angular.z = desired_pan_ - fromServoFrame(curr_pos);
			head_error_pub.publish(twist);
			
			std_msgs::Float64 f;
			f.data = curr_curr;
			head_current_pub.publish(f);
		}
		else {
			ROS_ERROR_THROTTLE(1,"[head_driver] Couldn't read servo position");
		}
		
		// Send the commanded angle to the servo
		//valid range is -23 to 232, but for most motors ~30-210
		ROS_DEBUG_THROTTLE(1,"[head_driver] Commanded angle %.2f (%.2f in servo frame)", desired_pan_, toServoFrame(desired_pan_));
		CPhidgetAdvancedServo_setPosition(servo, 0, toServoFrame(desired_pan_));
		
		// Publish a transform encorporating the actual position of the servo
		// No translation, one degree of rotation (pan).
		transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
		double corrected_pos = fromServoFrame(curr_pos) + curr_vel*feedforward_kv;
		transform.setRotation( tf::Quaternion(corrected_pos*pi/180, 0.0, 0.0) );
		br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame ));
		
		ROS_DEBUG_STREAM(boost::format("[head_driver] Position is %.3f, %.3f with FF") %fromServoFrame(curr_pos) %corrected_pos);
		
		// Allow callbacks to occur, and sleep to enforce the desired rate.
		loop_rate.sleep();
		ros::spinOnce();
		
		if( loop_rate.cycleTime() > loop_rate.expectedCycleTime() )
		{
			ROS_WARN_STREAM(boost::format("[head_driver] Missed update time of %.3fsec, took %.3fsec") % (loop_rate.expectedCycleTime().toSec()) %(loop_rate.cycleTime().toSec()) );
		}
	}
	
	CPhidgetAdvancedServo_setEngaged (servo, 0, 0);
	CPhidget_close ((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);
	
	ROS_INFO("[head_driver] Shutdown now.");
	return 0;
}
