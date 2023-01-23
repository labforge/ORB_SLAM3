;; Auto-generated. Do not edit!


(when (boundp 'orb_slam3_ros_wrapper::Point2f)
  (if (not (find-package "ORB_SLAM3_ROS_WRAPPER"))
    (make-package "ORB_SLAM3_ROS_WRAPPER"))
  (shadow 'Point2f (find-package "ORB_SLAM3_ROS_WRAPPER")))
(unless (find-package "ORB_SLAM3_ROS_WRAPPER::POINT2F")
  (make-package "ORB_SLAM3_ROS_WRAPPER::POINT2F"))

(in-package "ROS")
;;//! \htmlinclude Point2f.msg.html


(defclass orb_slam3_ros_wrapper::Point2f
  :super ros::object
  :slots (_x _y ))

(defmethod orb_slam3_ros_wrapper::Point2f
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get orb_slam3_ros_wrapper::Point2f :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get orb_slam3_ros_wrapper::Point2f :datatype-) "orb_slam3_ros_wrapper/Point2f")
(setf (get orb_slam3_ros_wrapper::Point2f :definition-)
      "#class cv::Point2f
#{
#    float x;
#    float y;
#}

float32 x
float32 y
")



(provide :orb_slam3_ros_wrapper/Point2f "ff8d7d66dd3e4b731ef14a45d38888b6")


