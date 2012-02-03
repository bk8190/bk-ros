; Auto-generated. Do not edit!


(cl:in-package mapping_msgs-msg)


;//! \htmlinclude CollisionMap.msg.html

(cl:defclass <CollisionMap> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (boxes
    :reader boxes
    :initarg :boxes
    :type (cl:vector mapping_msgs-msg:OrientedBoundingBox)
   :initform (cl:make-array 0 :element-type 'mapping_msgs-msg:OrientedBoundingBox :initial-element (cl:make-instance 'mapping_msgs-msg:OrientedBoundingBox))))
)

(cl:defclass CollisionMap (<CollisionMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CollisionMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CollisionMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapping_msgs-msg:<CollisionMap> is deprecated: use mapping_msgs-msg:CollisionMap instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CollisionMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:header-val is deprecated.  Use mapping_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'boxes-val :lambda-list '(m))
(cl:defmethod boxes-val ((m <CollisionMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:boxes-val is deprecated.  Use mapping_msgs-msg:boxes instead.")
  (boxes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CollisionMap>) ostream)
  "Serializes a message object of type '<CollisionMap>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'boxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'boxes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CollisionMap>) istream)
  "Deserializes a message object of type '<CollisionMap>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'boxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'boxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mapping_msgs-msg:OrientedBoundingBox))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CollisionMap>)))
  "Returns string type for a message object of type '<CollisionMap>"
  "mapping_msgs/CollisionMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CollisionMap)))
  "Returns string type for a message object of type 'CollisionMap"
  "mapping_msgs/CollisionMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CollisionMap>)))
  "Returns md5sum for a message object of type '<CollisionMap>"
  "18ca54db41ccebbe82f61f9801dede89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CollisionMap)))
  "Returns md5sum for a message object of type 'CollisionMap"
  "18ca54db41ccebbe82f61f9801dede89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CollisionMap>)))
  "Returns full string definition for message of type '<CollisionMap>"
  (cl:format cl:nil "#header for interpreting box positions~%Header header~%~%#boxes for use in collision testing~%OrientedBoundingBox[] boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mapping_msgs/OrientedBoundingBox~%#the center of the box~%geometry_msgs/Point32 center~%~%#the extents of the box, assuming the center is at the point~%geometry_msgs/Point32 extents~%~%#the axis of the box~%geometry_msgs/Point32 axis~%~%#the angle of rotation around the axis~%float32 angle~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CollisionMap)))
  "Returns full string definition for message of type 'CollisionMap"
  (cl:format cl:nil "#header for interpreting box positions~%Header header~%~%#boxes for use in collision testing~%OrientedBoundingBox[] boxes~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mapping_msgs/OrientedBoundingBox~%#the center of the box~%geometry_msgs/Point32 center~%~%#the extents of the box, assuming the center is at the point~%geometry_msgs/Point32 extents~%~%#the axis of the box~%geometry_msgs/Point32 axis~%~%#the angle of rotation around the axis~%float32 angle~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CollisionMap>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'boxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CollisionMap>))
  "Converts a ROS message object to a list"
  (cl:list 'CollisionMap
    (cl:cons ':header (header msg))
    (cl:cons ':boxes (boxes msg))
))
