;; Auto-generated. Do not edit!


(when (boundp 'orb_slam3_ros_wrapper::GlobalDescriptor)
  (if (not (find-package "ORB_SLAM3_ROS_WRAPPER"))
    (make-package "ORB_SLAM3_ROS_WRAPPER"))
  (shadow 'GlobalDescriptor (find-package "ORB_SLAM3_ROS_WRAPPER")))
(unless (find-package "ORB_SLAM3_ROS_WRAPPER::GLOBALDESCRIPTOR")
  (make-package "ORB_SLAM3_ROS_WRAPPER::GLOBALDESCRIPTOR"))

(in-package "ROS")
;;//! \htmlinclude GlobalDescriptor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass orb_slam3_ros_wrapper::GlobalDescriptor
  :super ros::object
  :slots (_header _data ))

(defmethod orb_slam3_ros_wrapper::GlobalDescriptor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get orb_slam3_ros_wrapper::GlobalDescriptor :md5sum-) "8903b686ebe5db3477e83c6d0bb149f8")
(setf (get orb_slam3_ros_wrapper::GlobalDescriptor :datatype-) "orb_slam3_ros_wrapper/GlobalDescriptor")
(setf (get orb_slam3_ros_wrapper::GlobalDescriptor :definition-)
      "Header header

# compressed global descriptor
# use rtabmap::util3d::uncompressData() from \"rtabmap/core/util3d.h\"
#int32 type
#uint8[] info
uint8[] data
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :orb_slam3_ros_wrapper/GlobalDescriptor "8903b686ebe5db3477e83c6d0bb149f8")


