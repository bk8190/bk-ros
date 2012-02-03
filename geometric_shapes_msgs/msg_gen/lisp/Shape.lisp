; Auto-generated. Do not edit!


(cl:in-package geometric_shapes_msgs-msg)


;//! \htmlinclude Shape.msg.html

(cl:defclass <Shape> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (dimensions
    :reader dimensions
    :initarg :dimensions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (triangles
    :reader triangles
    :initarg :triangles
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (vertices
    :reader vertices
    :initarg :vertices
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass Shape (<Shape>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Shape>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Shape)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name geometric_shapes_msgs-msg:<Shape> is deprecated: use geometric_shapes_msgs-msg:Shape instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Shape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geometric_shapes_msgs-msg:type-val is deprecated.  Use geometric_shapes_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'dimensions-val :lambda-list '(m))
(cl:defmethod dimensions-val ((m <Shape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geometric_shapes_msgs-msg:dimensions-val is deprecated.  Use geometric_shapes_msgs-msg:dimensions instead.")
  (dimensions m))

(cl:ensure-generic-function 'triangles-val :lambda-list '(m))
(cl:defmethod triangles-val ((m <Shape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geometric_shapes_msgs-msg:triangles-val is deprecated.  Use geometric_shapes_msgs-msg:triangles instead.")
  (triangles m))

(cl:ensure-generic-function 'vertices-val :lambda-list '(m))
(cl:defmethod vertices-val ((m <Shape>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader geometric_shapes_msgs-msg:vertices-val is deprecated.  Use geometric_shapes_msgs-msg:vertices instead.")
  (vertices m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Shape>)))
    "Constants for message type '<Shape>"
  '((:SPHERE . 0)
    (:BOX . 1)
    (:CYLINDER . 2)
    (:MESH . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Shape)))
    "Constants for message type 'Shape"
  '((:SPHERE . 0)
    (:BOX . 1)
    (:CYLINDER . 2)
    (:MESH . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Shape>) ostream)
  "Serializes a message object of type '<Shape>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dimensions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'dimensions))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'triangles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'triangles))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vertices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vertices))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Shape>) istream)
  "Deserializes a message object of type '<Shape>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dimensions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dimensions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'triangles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'triangles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vertices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vertices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Shape>)))
  "Returns string type for a message object of type '<Shape>"
  "geometric_shapes_msgs/Shape")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Shape)))
  "Returns string type for a message object of type 'Shape"
  "geometric_shapes_msgs/Shape")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Shape>)))
  "Returns md5sum for a message object of type '<Shape>"
  "59935940044147de75e7523b5d37c4d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Shape)))
  "Returns md5sum for a message object of type 'Shape"
  "59935940044147de75e7523b5d37c4d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Shape>)))
  "Returns full string definition for message of type '<Shape>"
  (cl:format cl:nil "byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Shape)))
  "Returns full string definition for message of type 'Shape"
  (cl:format cl:nil "byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Shape>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dimensions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'triangles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vertices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Shape>))
  "Converts a ROS message object to a list"
  (cl:list 'Shape
    (cl:cons ':type (type msg))
    (cl:cons ':dimensions (dimensions msg))
    (cl:cons ':triangles (triangles msg))
    (cl:cons ':vertices (vertices msg))
))
