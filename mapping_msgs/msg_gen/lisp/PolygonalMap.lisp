; Auto-generated. Do not edit!


(cl:in-package mapping_msgs-msg)


;//! \htmlinclude PolygonalMap.msg.html

(cl:defclass <PolygonalMap> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (polygons
    :reader polygons
    :initarg :polygons
    :type (cl:vector geometry_msgs-msg:Polygon)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Polygon :initial-element (cl:make-instance 'geometry_msgs-msg:Polygon)))
   (chan
    :reader chan
    :initarg :chan
    :type (cl:vector sensor_msgs-msg:ChannelFloat32)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:ChannelFloat32 :initial-element (cl:make-instance 'sensor_msgs-msg:ChannelFloat32))))
)

(cl:defclass PolygonalMap (<PolygonalMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolygonalMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolygonalMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapping_msgs-msg:<PolygonalMap> is deprecated: use mapping_msgs-msg:PolygonalMap instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PolygonalMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:header-val is deprecated.  Use mapping_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'polygons-val :lambda-list '(m))
(cl:defmethod polygons-val ((m <PolygonalMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:polygons-val is deprecated.  Use mapping_msgs-msg:polygons instead.")
  (polygons m))

(cl:ensure-generic-function 'chan-val :lambda-list '(m))
(cl:defmethod chan-val ((m <PolygonalMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:chan-val is deprecated.  Use mapping_msgs-msg:chan instead.")
  (chan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolygonalMap>) ostream)
  "Serializes a message object of type '<PolygonalMap>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'polygons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'polygons))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'chan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'chan))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolygonalMap>) istream)
  "Deserializes a message object of type '<PolygonalMap>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'polygons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'polygons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Polygon))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'chan) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'chan)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:ChannelFloat32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolygonalMap>)))
  "Returns string type for a message object of type '<PolygonalMap>"
  "mapping_msgs/PolygonalMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolygonalMap)))
  "Returns string type for a message object of type 'PolygonalMap"
  "mapping_msgs/PolygonalMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolygonalMap>)))
  "Returns md5sum for a message object of type '<PolygonalMap>"
  "3ef041ea9abaaa9dac3e5aec60c68a99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolygonalMap)))
  "Returns md5sum for a message object of type 'PolygonalMap"
  "3ef041ea9abaaa9dac3e5aec60c68a99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolygonalMap>)))
  "Returns full string definition for message of type '<PolygonalMap>"
  (cl:format cl:nil "Header header~%geometry_msgs/Polygon[] polygons~%sensor_msgs/ChannelFloat32[] chan~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%geometry_msgs/Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: sensor_msgs/ChannelFloat32~%# This message is used by the PointCloud message to hold optional data~%# associated with each point in the cloud. The length of the values~%# array should be the same as the length of the points array in the~%# PointCloud, and each value should be associated with the corresponding~%# point.~%~%# Channel names in existing practice include:~%#   \"u\", \"v\" - row and column (respectively) in the left stereo image.~%#              This is opposite to usual conventions but remains for~%#              historical reasons. The newer PointCloud2 message has no~%#              such problem.~%#   \"rgb\" - For point clouds produced by color stereo cameras. uint8~%#           (R,G,B) values packed into the least significant 24 bits,~%#           in order.~%#   \"intensity\" - laser or pixel intensity.~%#   \"distance\"~%~%# The channel name should give semantics of the channel (e.g.~%# \"intensity\" instead of \"value\").~%string name~%~%# The values array should be 1-1 with the elements of the associated~%# PointCloud.~%float32[] values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolygonalMap)))
  "Returns full string definition for message of type 'PolygonalMap"
  (cl:format cl:nil "Header header~%geometry_msgs/Polygon[] polygons~%sensor_msgs/ChannelFloat32[] chan~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%geometry_msgs/Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: sensor_msgs/ChannelFloat32~%# This message is used by the PointCloud message to hold optional data~%# associated with each point in the cloud. The length of the values~%# array should be the same as the length of the points array in the~%# PointCloud, and each value should be associated with the corresponding~%# point.~%~%# Channel names in existing practice include:~%#   \"u\", \"v\" - row and column (respectively) in the left stereo image.~%#              This is opposite to usual conventions but remains for~%#              historical reasons. The newer PointCloud2 message has no~%#              such problem.~%#   \"rgb\" - For point clouds produced by color stereo cameras. uint8~%#           (R,G,B) values packed into the least significant 24 bits,~%#           in order.~%#   \"intensity\" - laser or pixel intensity.~%#   \"distance\"~%~%# The channel name should give semantics of the channel (e.g.~%# \"intensity\" instead of \"value\").~%string name~%~%# The values array should be 1-1 with the elements of the associated~%# PointCloud.~%float32[] values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolygonalMap>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'polygons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'chan) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolygonalMap>))
  "Converts a ROS message object to a list"
  (cl:list 'PolygonalMap
    (cl:cons ':header (header msg))
    (cl:cons ':polygons (polygons msg))
    (cl:cons ':chan (chan msg))
))
