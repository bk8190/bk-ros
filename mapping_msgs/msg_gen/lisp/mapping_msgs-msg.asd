
(cl:in-package :asdf)

(defsystem "mapping_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometric_shapes_msgs-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CollisionObjectOperation" :depends-on ("_package_CollisionObjectOperation"))
    (:file "_package_CollisionObjectOperation" :depends-on ("_package"))
    (:file "PolygonalMap" :depends-on ("_package_PolygonalMap"))
    (:file "_package_PolygonalMap" :depends-on ("_package"))
    (:file "CollisionMap" :depends-on ("_package_CollisionMap"))
    (:file "_package_CollisionMap" :depends-on ("_package"))
    (:file "OrientedBoundingBox" :depends-on ("_package_OrientedBoundingBox"))
    (:file "_package_OrientedBoundingBox" :depends-on ("_package"))
    (:file "AttachedCollisionObject" :depends-on ("_package_AttachedCollisionObject"))
    (:file "_package_AttachedCollisionObject" :depends-on ("_package"))
    (:file "CollisionObject" :depends-on ("_package_CollisionObject"))
    (:file "_package_CollisionObject" :depends-on ("_package"))
  ))