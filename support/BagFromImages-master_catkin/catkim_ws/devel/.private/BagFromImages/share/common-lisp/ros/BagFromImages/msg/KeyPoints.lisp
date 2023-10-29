; Auto-generated. Do not edit!


(cl:in-package BagFromImages-msg)


;//! \htmlinclude KeyPoints.msg.html

(cl:defclass <KeyPoints> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (keypoints
    :reader keypoints
    :initarg :keypoints
    :type (cl:vector BagFromImages-msg:KeyPoint)
   :initform (cl:make-array 0 :element-type 'BagFromImages-msg:KeyPoint :initial-element (cl:make-instance 'BagFromImages-msg:KeyPoint))))
)

(cl:defclass KeyPoints (<KeyPoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyPoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyPoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name BagFromImages-msg:<KeyPoints> is deprecated: use BagFromImages-msg:KeyPoints instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <KeyPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BagFromImages-msg:header-val is deprecated.  Use BagFromImages-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'keypoints-val :lambda-list '(m))
(cl:defmethod keypoints-val ((m <KeyPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BagFromImages-msg:keypoints-val is deprecated.  Use BagFromImages-msg:keypoints instead.")
  (keypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyPoints>) ostream)
  "Serializes a message object of type '<KeyPoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'keypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'keypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyPoints>) istream)
  "Deserializes a message object of type '<KeyPoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'keypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'keypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'BagFromImages-msg:KeyPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyPoints>)))
  "Returns string type for a message object of type '<KeyPoints>"
  "BagFromImages/KeyPoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyPoints)))
  "Returns string type for a message object of type 'KeyPoints"
  "BagFromImages/KeyPoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyPoints>)))
  "Returns md5sum for a message object of type '<KeyPoints>"
  "ada03370f65713b6c35c9e1949b83815")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyPoints)))
  "Returns md5sum for a message object of type 'KeyPoints"
  "ada03370f65713b6c35c9e1949b83815")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyPoints>)))
  "Returns full string definition for message of type '<KeyPoints>"
  (cl:format cl:nil "Header header~%~%KeyPoint[] keypoints~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: BagFromImages/KeyPoint~%#class cv::KeyPoint~%#{~%#    Point2f pt;~%#    float size;~%#    float angle;~%#    float response;~%#    int octave;~%#    int class_id;~%#}~%~%Point2f pt~%float32 size~%float32 angle~%float32 response~%int32 octave~%int32 class_id~%================================================================================~%MSG: BagFromImages/Point2f~%#class cv::Point2f~%#{~%#    float x;~%#    float y;~%#}~%~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyPoints)))
  "Returns full string definition for message of type 'KeyPoints"
  (cl:format cl:nil "Header header~%~%KeyPoint[] keypoints~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: BagFromImages/KeyPoint~%#class cv::KeyPoint~%#{~%#    Point2f pt;~%#    float size;~%#    float angle;~%#    float response;~%#    int octave;~%#    int class_id;~%#}~%~%Point2f pt~%float32 size~%float32 angle~%float32 response~%int32 octave~%int32 class_id~%================================================================================~%MSG: BagFromImages/Point2f~%#class cv::Point2f~%#{~%#    float x;~%#    float y;~%#}~%~%float32 x~%float32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyPoints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'keypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyPoints>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyPoints
    (cl:cons ':header (header msg))
    (cl:cons ':keypoints (keypoints msg))
))
