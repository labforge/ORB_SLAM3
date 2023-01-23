
(cl:in-package :asdf)

(defsystem "orb_slam3_ros_wrapper-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Descriptor" :depends-on ("_package_Descriptor"))
    (:file "_package_Descriptor" :depends-on ("_package"))
    (:file "GlobalDescriptor" :depends-on ("_package_GlobalDescriptor"))
    (:file "_package_GlobalDescriptor" :depends-on ("_package"))
    (:file "KeyPoint" :depends-on ("_package_KeyPoint"))
    (:file "_package_KeyPoint" :depends-on ("_package"))
    (:file "KeyPoints" :depends-on ("_package_KeyPoints"))
    (:file "_package_KeyPoints" :depends-on ("_package"))
    (:file "Point2f" :depends-on ("_package_Point2f"))
    (:file "_package_Point2f" :depends-on ("_package"))
  ))