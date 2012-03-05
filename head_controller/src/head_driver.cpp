#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <phidget21.h>

double desired_pan_ = 0.0;
double actual_pan_ = 0.0;

void panAngleCallback(const std_msgs::Float64& msg)
{
	// TODO: Impose reasonable limits on the angle, or transform it to center it

	desired_pan_ = msg.data;
}

int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	ROS_INFO("[head_driver] %s %10d attached!\n", name, serialNo);

	return 0;
}

int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	ROS_INFO("[head_driver] %s %10d detached!\n", name, serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{
	ROS_ERROR("[head_driver] Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
	actual_pan_ = Value;
	ROS_INFO("[head_driver] Motor: %d > Current Position: %f\n", Index, Value);
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
int display_properties(CPhidgetAdvancedServoHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);

	ROS_INFO("%s\n", ptr);
	ROS_INFO("[head_driver] Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

	return 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "head_driver");
	ros::NodeHandle nh;
	tf::TransformBroadcaster br;
	ros::Subscriber angle_sub_ = nh.subscribe("/pan_command", 1, &panAngleCallback);
	
	// We control a servo that determines the link between parent_frame and child_frame
	std::string parent_frame, child_frame;
	nh.param<std::string>("parent_tf_frame", parent_frame , "NULL");
	nh.param<std::string>("child_tf_frame" , child_frame  , "NULL");
	ROS_INFO("[head_driver] Broadcasting TF from \"%s\" to \"%s\"",
		parent_frame.c_str(), child_frame.c_str());
	
	// Constraints on pan velocity and acceleration
	double pan_vel_max, pan_acc_max;
	nh.param("pan_vel_max", pan_vel_max, 0.5);
	nh.param("pan_acc_max", pan_acc_max, 0.5);
	ROS_INFO("[head_driver] Accel: %.2f, Velocity: %.2f", pan_acc_max, pan_vel_max);
	
	// Control loop rate
	double loop_rate_dbl;
	nh.param("loop_rate", loop_rate_dbl, 10.0);
	ros::Rate loop_rate = ros::Rate(loop_rate_dbl);
	ROS_INFO("[head_driver] Loop rate is %.2fHz", loop_rate_dbl);	
	
	
	//Declare an advanced servo handle
	CPhidgetAdvancedServoHandle servo = 0;

	//create the advanced servo object
	CPhidgetAdvancedServo_create(&servo);
	
	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
	CPhidget_set_OnError_Handler ((CPhidgetHandle)servo, ErrorHandler , NULL);
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);
	
	//open the device for connections
	CPhidget_open((CPhidgetHandle)servo, -1);

	int result;
	const char *err;
	
	// Wait for the servo to be attached
	ROS_INFO("Waiting for Phidget to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		ROS_INFO("Problem waiting for attachment: %s\n", err);
		return 0;
	}
	
	//Display the properties of the attached device
	display_properties(servo);
	
	//Set up some initial acceleration and velocity values
	// TODO: Currently, is overwritng with the defaults.
	CPhidgetAdvancedServo_getAccelerationMin(servo, 0, &pan_acc_max);
	CPhidgetAdvancedServo_setAcceleration   (servo, 0, pan_acc_max);
	CPhidgetAdvancedServo_getVelocityMax    (servo, 0, &pan_vel_max);
	CPhidgetAdvancedServo_setVelocityLimit  (servo, 0, pan_vel_max);
	ROS_INFO("[head_driver] Accel: %.2f, Velocity: %.2f", pan_acc_max, pan_vel_max);
	
	// Center the servo, engage the drive.
	CPhidgetAdvancedServo_setPosition(servo, 0, 0.0);
	CPhidgetAdvancedServo_setEngaged (servo, 0, 1);
	
	ros::Duration(1.0).sleep();
	
	double curr_pos;
	tf::Transform transform;
	
	while( ros::ok() )
	{
		//Get current motor position
		if(CPhidgetAdvancedServo_getPosition(servo, 0, &curr_pos) == EPHIDGET_OK)
		{
			ROS_INFO_THROTTLE(2,"[head_driver] Motor: 0 > Current Position: %f\n", curr_pos);
		
			// Send the commanded angle to the servo
			//valid range is -23 to 232, but for most motors ~30-210
			ROS_INFO_THROTTLE(2, "[head_driver] Commanded angle %.2f", desired_pan_);
			CPhidgetAdvancedServo_setPosition(servo, 0, desired_pan_);
		
			// TODO: This is temporary, and will be replaced when the servo is hooked up
			actual_pan_ = desired_pan_;
		
			// Publish a transform encorporating the actual position of the servo
			// No translation, one degree of rotation (pan).
			transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
			transform.setRotation( tf::Quaternion(actual_pan_, 0.0, 0.0) );
			br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame ));
		}
		else
		{
			ROS_ERROR_THROTTLE(2,"[head_driver] Could not read servo position");
		}
		
	/*
		// TODO: PD control loop on the desired angle
		cmd_pan = desired_pan_;
		
		// TODO: Send the commanded angle to the servo
		ROS_INFO_THROTTLE(2, "[head_driver] Commanded angle %.2f", desired_pan_);
	
		// Create and publish a transform. No translation, one degree of rotation (pan).
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
		transform.setRotation( tf::Quaternion(actual_pan_, 0.0, 0.0) );
		br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame ));
	*/
	
		// Allow callbacks to occur, and sleep to enforce the desired rate.
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	CPhidget_close ((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);
	
	ROS_INFO("[head_driver] Shutdown now.");
	return 0;
}
