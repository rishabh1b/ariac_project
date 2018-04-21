
(cl:in-package :asdf)

(defsystem "localisation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "request_logical_pose" :depends-on ("_package_request_logical_pose"))
    (:file "_package_request_logical_pose" :depends-on ("_package"))
  ))