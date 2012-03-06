#! /usr/bin/env python

import roslib; roslib.load_manifest('head_controller')
import rospy
import tf
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

import math

class PointHeadNode():

	def __init__(self):
		# Initialize node
		rospy.init_node('point_head_node', anonymous=True)
		
		my_namespace = rospy.get_namespace()
		rate = rospy.get_param('~rate', 1)
		rospy.loginfo('[point_head] Rate = ' + str(rate))
		r = rospy.Rate(rate)
		
		# Initialize the target points
		self.target_point      = PointStamped()
		self.last_target_point = PointStamped()

		# Subscribe to the target_pose topic
		rospy.Subscriber('/target_pose', PoseStamped, self.update_target_point)

		# Initialize publisher for the pan servo
		self.head_pan_frame = 'pan_link'
		self.head_pan_pub = rospy.Publisher(my_namespace + 'pan_command', Float64)
		
		# Initialize tf listener
		self.tf = tf.TransformListener()
	
		# Reset the head position to neutral
		rospy.sleep(1)
		self.reset_head_position()
		rospy.loginfo("[point_head] Ready to accept target point")
		
		while not rospy.is_shutdown():
			rospy.wait_for_message('/target_pose', PoseStamped)
			if self.target_point == self.last_target_point:
				rospy.loginfo("[point_head] Old target point")
				continue
			
			try:
				target_pan = self.transform_target_point(self.target_point)
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("[point_head] tf Failure")
				continue

			rospy.loginfo("[point_head] Setting Target pan:\n" + str(target_pan))
			
			self.head_pan_pub.publish(target_pan)
			self.last_target_point = self.target_point
			r.sleep()


	def update_target_point(self, msg):
		rospy.loginfo("[point_head] Got new pose in frame " + msg.header.frame_id + ":\n" +  str(msg.pose.position))
		self.target_point = PointStamped()
		self.target_point.point  = msg.pose.position
		self.target_point.header = msg.header


	def reset_head_position(self):
		self.head_pan_pub.publish(0.0)
		rospy.sleep(2)


	def transform_target_point(self, target):
		# Set the pan reference frame to the head_pan_frame defined above
		pan_ref_frame = self.head_pan_frame
		
		# Wait for tf info (time-out in 5 seconds)
		self.tf.waitForTransform(pan_ref_frame, target.header.frame_id, rospy.Time(), \
	rospy.Duration(5.0))

		# Transform target point to pan reference frame & retrieve the pan angle
		pan_target = self.tf.transformPoint(pan_ref_frame, target)
		pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)

		return pan_angle


if __name__ == '__main__':
	try:
		point_head = PointHeadNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	
	rospy.loginfo("[point_head] Node shutting down.")
