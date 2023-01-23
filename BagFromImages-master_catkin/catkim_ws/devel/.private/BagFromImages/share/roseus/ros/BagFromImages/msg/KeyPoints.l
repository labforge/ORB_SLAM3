;; Auto-generated. Do not edit!


(when (boundp 'BagFromImages::KeyPoints)
  (if (not (find-package "BAGFROMIMAGES"))
    (make-package "BAGFROMIMAGES"))
  (shadow 'KeyPoints (find-package "BAGFROMIMAGES")))
(unless (find-package "BAGFROMIMAGES::KEYPOINTS")
  (make-package "BAGFROMIMAGES::KEYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude KeyPoints.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass BagFromImages::KeyPoints
  :super ros::object
  :slots (_header _keypoints ))

(defmethod BagFromImages::KeyPoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:keypoints __keypoints) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _keypoints __keypoints)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:keypoints
   (&rest __keypoints)
   (if (keywordp (car __keypoints))
       (send* _keypoints __keypoints)
     (progn
       (if __keypoints (setq _keypoints (car __keypoints)))
       _keypoints)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; BagFromImages/KeyPoint[] _keypoints
    (apply #'+ (send-all _keypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; BagFromImages/KeyPoint[] _keypoints
     (write-long (length _keypoints) s)
     (dolist (elem _keypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; BagFromImages/KeyPoint[] _keypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keypoints (let (r) (dotimes (i n) (push (instance BagFromImages::KeyPoint :init) r)) r))
     (dolist (elem- _keypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get BagFromImages::KeyPoints :md5sum-) "ada03370f65713b6c35c9e1949b83815")
(setf (get BagFromImages::KeyPoints :datatype-) "BagFromImages/KeyPoints")
(setf (get BagFromImages::KeyPoints :definition-)
      "Header header

KeyPoint[] keypoints
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

================================================================================
MSG: BagFromImages/KeyPoint
#class cv::KeyPoint
#{
#    Point2f pt;
#    float size;
#    float angle;
#    float response;
#    int octave;
#    int class_id;
#}

Point2f pt
float32 size
float32 angle
float32 response
int32 octave
int32 class_id
================================================================================
MSG: BagFromImages/Point2f
#class cv::Point2f
#{
#    float x;
#    float y;
#}

float32 x
float32 y
")



(provide :BagFromImages/KeyPoints "ada03370f65713b6c35c9e1949b83815")


