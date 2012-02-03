; Auto-generated. Do not edit!


(cl:in-package mapping_msgs-msg)


;//! \htmlinclude CollisionObjectOperation.msg.html

(cl:defclass <CollisionObjectOperation> (roslisp-msg-protocol:ros-message)
  ((operation
    :reader operation
    :initarg :operation
    :type cl:integer
    :initform 0))
)

(cl:defclass CollisionObjectOperation (<CollisionObjectOperation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CollisionObjectOperation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CollisionObjectOperation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mapping_msgs-msg:<CollisionObjectOperation> is deprecated: use mapping_msgs-msg:CollisionObjectOperation instead.")))

(cl:ensure-generic-function 'operation-val :lambda-list '(m))
(cl:defmethod operation-val ((m <CollisionObjectOperation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mapping_msgs-msg:operation-val is deprecated.  Use mapping_msgs-msg:operation instead.")
  (operation m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CollisionObjectOperation>)))
    "Constants for message type '<CollisionObjectOperation>"
  '((:ADD . 0)
    (:REMOVE . 1)
    (:DETACH_AND_ADD_AS_OBJECT . 2)
    (:ATTACH_AND_REMOVE_AS_OBJECT . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CollisionObjectOperation)))
    "Constants for message type 'CollisionObjectOperation"
  '((:ADD . 0)
    (:REMOVE . 1)
    (:DETACH_AND_ADD_AS_OBJECT . 2)
    (:ATTACH_AND_REMOVE_AS_OBJECT . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CollisionObjectOperation>) ostream)
  "Serializes a message object of type '<CollisionObjectOperation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'operation)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CollisionObjectOperation>) istream)
  "Deserializes a message object of type '<CollisionObjectOperation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'operation)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CollisionObjectOperation>)))
  "Returns string type for a message object of type '<CollisionObjectOperation>"
  "mapping_msgs/CollisionObjectOperation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CollisionObjectOperation)))
  "Returns string type for a message object of type 'CollisionObjectOperation"
  "mapping_msgs/CollisionObjectOperation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CollisionObjectOperation>)))
  "Returns md5sum for a message object of type '<CollisionObjectOperation>"
  "66a2b3b971d193145f8da8c3e401a474")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CollisionObjectOperation)))
  "Returns md5sum for a message object of type 'CollisionObjectOperation"
  "66a2b3b971d193145f8da8c3e401a474")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CollisionObjectOperation>)))
  "Returns full string definition for message of type '<CollisionObjectOperation>"
  (cl:format cl:nil "#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CollisionObjectOperation)))
  "Returns full string definition for message of type 'CollisionObjectOperation"
  (cl:format cl:nil "#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CollisionObjectOperation>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CollisionObjectOperation>))
  "Converts a ROS message object to a list"
  (cl:list 'CollisionObjectOperation
    (cl:cons ':operation (operation msg))
))
