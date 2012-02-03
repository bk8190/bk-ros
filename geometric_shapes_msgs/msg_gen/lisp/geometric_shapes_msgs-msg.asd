
(cl:in-package :asdf)

(defsystem "geometric_shapes_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Shape" :depends-on ("_package_Shape"))
    (:file "_package_Shape" :depends-on ("_package"))
  ))