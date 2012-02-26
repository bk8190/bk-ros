time stamp
int32 seq    # seq is used for tracking the hand - 
			 # if the seq number is the same, you can assume that the hand is the same as previously seen
int32 thumb
bool left    # if this hand is the left hand
geometry_msgs/Point arm
geometry_msgs/Transform palm
geometry_msgs/Point[] fingers
sensor_msgs/PointCloud2 handcloud
#Possibilities for state variable:
# open - open palm, usually five fingers
# grip - fingers curled forward
# paddle -  fingers together and straight
# fist   
string state
