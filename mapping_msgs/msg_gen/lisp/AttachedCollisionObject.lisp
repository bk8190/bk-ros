; Auto-generated. Do not edit!


(cl:in-package mapping_msgs-msg)


;//! \htmlinclude AttachedCollisionObject.msg.html

(cl:defclass <AttachedCollisionObject> (roslisp-msg-protocol:ros-message)
  ((link_name
    :reader link_name
    :initarg :link_name
    :type cl:string
    :initform "")
   (object
    :reader object
    :initarg :object
    :type mapping_msgs-msg:CollisionObject
    :initform (cl:make-instance 'mapping_msgs-msg:CollisionObject))
   (touch_links
    :reader touch_links
    :initarg :touch_links
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass AttachedCollisionObject (<AttachedCollisionObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttachedCollisionObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttachedCollisionObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapping_msgs-msg:<AttachedCollisionObject> is deprecated: use mapping_msgs-msg:AttachedCollisionObject instead.")))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <AttachedCollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:link_name-val is deprecated.  Use mapping_msgs-msg:link_name instead.")
  (link_name m))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <AttachedCollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:object-val is deprecated.  Use mapping_msgs-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'touch_links-val :lambda-list '(m))
(cl:defmethod touch_links-val ((m <AttachedCollisionObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:touch_links-val is deprecated.  Use mapping_msgs-msg:touch_links instead.")
  (touch_links m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AttachedCollisionObject>)))
    "Constants for message type '<AttachedCollisionObject>"
  '((:REMOVE_ALL_ATTACHED_OBJECTS . "all"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AttachedCollisionObject)))
    "Constants for message type 'AttachedCollisionObject"
  '((:REMOVE_ALL_ATTACHED_OBJECTS . "all"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttachedCollisionObject>) ostream)
  "Serializes a message object of type '<AttachedCollisionObject>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'link_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'link_name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'touch_links))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'touch_links))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttachedCollisionObject>) istream)
  "Deserializes a message object of type '<AttachedCollisionObject>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'link_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'touch_links) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'touch_links)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttachedCollisionObject>)))
  "Returns string type for a message object of type '<AttachedCollisionObject>"
  "mapping_msgs/AttachedCollisionObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttachedCollisionObject)))
  "Returns string type for a message object of type 'AttachedCollisionObject"
  "mapping_msgs/AttachedCollisionObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttachedCollisionObject>)))
  "Returns md5sum for a message object of type '<AttachedCollisionObject>"
  "58c7f119e35988da1dbd450c682308ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttachedCollisionObject)))
  "Returns md5sum for a message object of type 'AttachedCollisionObject"
  "58c7f119e35988da1dbd450c682308ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttachedCollisionObject>)))
  "Returns full string definition for message of type '<AttachedCollisionObject>"
  (cl:format cl:nil "# The CollisionObject will be attached with a fixed joint to this link~%# If link name is set to REMOVE_ALL_ATTACHED_OBJECTS and object.operation ~%# is set to REMOVE will remove all attached bodies attached to any object~%string link_name~%~%#Reserved for indicating that all attached objects should be removed~%string REMOVE_ALL_ATTACHED_OBJECTS = \"all\"~%~%#This contains the actual shapes and poses for the CollisionObject~%#to be attached to the link~%#If action is remove and no object.id is set, all objects~%#attached to the link indicated by link_name will be removed~%CollisionObject object~%~%# The set of links that the attached objects are allowed to touch~%# by default - the link_name is included by default~%string[] touch_links~%~%================================================================================~%MSG: mapping_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%geometric_shapes_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mapping_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: geometric_shapes_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttachedCollisionObject)))
  "Returns full string definition for message of type 'AttachedCollisionObject"
  (cl:format cl:nil "# The CollisionObject will be attached with a fixed joint to this link~%# If link name is set to REMOVE_ALL_ATTACHED_OBJECTS and object.operation ~%# is set to REMOVE will remove all attached bodies attached to any object~%string link_name~%~%#Reserved for indicating that all attached objects should be removed~%string REMOVE_ALL_ATTACHED_OBJECTS = \"all\"~%~%#This contains the actual shapes and poses for the CollisionObject~%#to be attached to the link~%#If action is remove and no object.id is set, all objects~%#attached to the link indicated by link_name will be removed~%CollisionObject object~%~%# The set of links that the attached objects are allowed to touch~%# by default - the link_name is included by default~%string[] touch_links~%~%================================================================================~%MSG: mapping_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%geometric_shapes_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mapping_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: geometric_shapes_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttachedCollisionObject>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'link_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'touch_links) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttachedCollisionObject>))
  "Converts a ROS message object to a list"
  (cl:list 'AttachedCollisionObject
    (cl:cons ':link_name (link_name msg))
    (cl:cons ':object (object msg))
    (cl:cons ':touch_links (touch_links msg))
))
