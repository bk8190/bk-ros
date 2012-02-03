#!/usr/bin/env python
import roslib
roslib.load_manifest('nifun')
import rospy


#import message types:
from mapping_msgs.msg import PolygonalMap
from geometry_msgs.msg import Twist

from math import  *

def getangle(scan,index):
    return scan.angle_min+scan.angle_increment*float(index)


def cmdstop():
    print "stopping"
    cmd = Twist()
    pub.publish(cmd)   

hadcallback=1    

    
def myskelCallback(pmap):
    global hadcallback
    hadcallback=0
    avex = 0.0
    avez = 0.0
    for p in pmap.polygons:
        avex+=p.points[0].x
        avez+=p.points[0].z
    if len(pmap.polygons) == 0:
        cmdstop()
        return
    avex/=len(pmap.polygons)
    avez/=len(pmap.polygons)
    
    print avex,avez
    cmd = Twist()
    #if the object is within .8 meters, turn to face
    if abs(avex) > .01:
       cmd.angular.z = avex *2.0
    
    if abs(avez - 1.50) > .1:
       cmd.linear.x = (avez-1.50)*1.0 
    
       
    #regardless of whether we set things, publish the command:
    pub.publish(cmd)   

    
#this command registers this process with the ros master
rospy.init_node('my_controller')

#register a publisher, to topic '/base_controller/command', of type Twist
pub = rospy.Publisher('/cmd_vel', Twist)
#register a callback for messages of type LaserScan, on the topic "/base_scan"
sub = rospy.Subscriber("/skeletons", PolygonalMap, myskelCallback)
cmdstop()

#this line is equivalent to: 
# while(everything is ok)
#   check for messages, call callbacks
#   sleep
#rospy.spin()

r = rospy.Rate(5) # 5hz
while not rospy.is_shutdown():
    hadcallback+=1
    if hadcallback > 1:
        print "Timeout exceded"
        cmdstop()
    r.sleep()




