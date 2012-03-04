#! /usr/bin/env python

import roslib; roslib.load_manifest('head_controller')
import rospy
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

import math

class PointHeadNode():

	def __init__(self):
		# Initialize node
		rospy.init_node('point_head_node', anonymous=True)
		
		my_namespace = rospy.get_namespace()
		rate = rospy.get_param('~rate', 1)
		print('Rate = ', rate)
		r = rospy.Rate(rate)
		
		# Initialize the target point
		self.target_point = PointStamped()
		self.last_target_point = PointStamped()

		# Subscribe to the target_point topic
		rospy.Subscriber('/target_point', PointStamped, self.update_target_point)

		# Initialize publisher for the pan servo
		self.head_pan_frame = 'head_pan_link'
		self.head_pan_pub = rospy.Publisher(my_namespace + 'pan_command', Float64)
		
		# Initialize tf listener
		self.tf = tf.TransformListener()
		
		# Make sure we can see at least the pan and tilt frames
		#self.tf.waitForTransform(self.head_pan_frame, self.head_tilt_frame, rospy.Time(), rospy.Duration(5.0))
		
		# Reset the head position to neutral
		rospy.sleep(1)
		self.reset_head_position()
		rospy.loginfo("Ready to accept target point")
		
		while not rospy.is_shutdown():
			rospy.wait_for_message('/target_point', PointStamped)
			if self.target_point == self.last_target_point:
				continue
			try:
				target_angles = self.transform_target_point(self.target_point)
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("tf Failure")
				continue

			self.head_pan_pub.publish(target_angles[0])
			#self.head_tilt_pub.publish(target_angles[1])

			self.last_target_point = self.target_point
			rospy.loginfo("Setting Target Point:\n" + str(self.target_point))

			r.sleep()
		
	def update_target_point(self, msg):
		self.target_point = msg

	def reset_head_position(self):
		self.head_pan_pub.publish(0.0)
		#self.head_tilt_pub.publish(0.0)
		rospy.sleep(3)

	def transform_target_point(self, target):
		# Set the pan and tilt reference frames to the head_pan_frame and head_tilt_frame defined above
		pan_ref_frame = self.head_pan_frame
		#tilt_ref_frame = self.head_tilt_frame
		
		# Wait for tf info (time-out in 5 seconds)
		self.tf.waitForTransform(pan_ref_frame, target.header.frame_id, rospy.Time(), \
	rospy.Duration(5.0))
		#self.tf.waitForTransform(tilt_ref_frame, target.header.frame_id, rospy.Time(), \
	#rospy.Duration(5.0))

		# Transform target point to pan reference frame & retrieve the pan angle
		pan_target = self.tf.transformPoint(pan_ref_frame, target)
		pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)

		# Transform target point to tilt reference frame & retrieve the tilt angle
		#tilt_target = self.tf.transformPoint(tilt_ref_frame, target)
		#tilt_angle = math.atan2(tilt_target.point.z,
		#        math.sqrt(math.pow(tilt_target.point.x, 2) + math.pow(tilt_target.point.y, 2)))

		return [pan_angle]#, tilt_angle]

if __name__ == '__main__':
	try:
		point_head = PointHeadNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
