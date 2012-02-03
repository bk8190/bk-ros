; Auto-generated. Do not edit!


(cl:in-package mapping_msgs-msg)


;//! \htmlinclude CollisionObject.msg.html

(cl:defclass <CollisionObject> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (operation
    :reader operation
    :initarg :operation
    :type mapping_msgs-msg:CollisionObjectOperation
    :initform (cl:make-instance 'mapping_msgs-msg:CollisionObjectOperation))
   (shapes
    :reader shapes
    :initarg :shapes
    :type (cl:vector geometric_shapes_msgs-msg:Shape)
   :initform (cl:make-array 0 :element-type 'geometric_shapes_msgs-msg:Shape :initial-element (cl:make-instance 'geometric_shapes_msgs-msg:Shape)))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass CollisionObject (<CollisionObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CollisionObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CollisionObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapping_msgs-msg:<CollisionObject> is deprecated: use mapping_msgs-msg:CollisionObject instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:header-val is deprecated.  Use mapping_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <CollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:id-val is deprecated.  Use mapping_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'operation-val :lambda-list '(m))
(cl:defmethod operation-val ((m <CollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:operation-val is deprecated.  Use mapping_msgs-msg:operation instead.")
  (operation m))

(cl:ensure-generic-function 'shapes-val :lambda-list '(m))
(cl:defmethod shapes-val ((m <CollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:shapes-val is deprecated.  Use mapping_msgs-msg:shapes instead.")
  (shapes m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <CollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:poses-val is deprecated.  Use mapping_msgs-msg:poses instead.")
  (poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CollisionObject>) ostream)
  "Serializes a message object of type '<CollisionObject>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'operation) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'shapes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'shapes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CollisionObject>) istream)
  "Deserializes a message object of type '<CollisionObject>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'operation) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'shapes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'shapes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometric_shapes_msgs-msg:Shape))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CollisionObject>)))
  "Returns string type for a message object of type '<CollisionObject>"
  "mapping_msgs/CollisionObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CollisionObject)))
  "Returns string type for a message object of type 'CollisionObject"
  "mapping_msgs/CollisionObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CollisionObject>)))
  "Returns md5sum for a message object of type '<CollisionObject>"
  "c25d22faff81b340d88e28e270ae03f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CollisionObject)))
  "Returns md5sum for a message object of type 'CollisionObject"
  "c25d22faff81b340d88e28e270ae03f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CollisionObject>)))
  "Returns full string definition for message of type '<CollisionObject>"
  (cl:format cl:nil "# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%geometric_shapes_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mapping_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: geometric_shapes_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CollisionObject)))
  "Returns full string definition for message of type 'CollisionObject"
  (cl:format cl:nil "# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%geometric_shapes_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mapping_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: geometric_shapes_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CollisionObject>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'operation))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'shapes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CollisionObject>))
  "Converts a ROS message object to a list"
  (cl:list 'CollisionObject
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':operation (operation msg))
    (cl:cons ':shapes (shapes msg))
    (cl:cons ':poses (poses msg))
))
