; Auto-generated. Do not edit!


(cl:in-package BagFromImages-msg)


;//! \htmlinclude Descriptor.msg.html

(cl:defclass <Descriptor> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rows
    :reader rows
    :initarg :rows
    :type cl:integer
    :initform 0)
   (cols
    :reader cols
    :initarg :cols
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (descdata
    :reader descdata
    :initarg :descdata
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Descriptor (<Descriptor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Descriptor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Descriptor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name BagFromImages-msg:<Descriptor> is deprecated: use BagFromImages-msg:Descriptor instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Descriptor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BagFromImages-msg:header-val is deprecated.  Use BagFromImages-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rows-val :lambda-list '(m))
(cl:defmethod rows-val ((m <Descriptor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BagFromImages-msg:rows-val is deprecated.  Use BagFromImages-msg:rows instead.")
  (rows m))

(cl:ensure-generic-function 'cols-val :lambda-list '(m))
(cl:defmethod cols-val ((m <Descriptor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BagFromImages-msg:cols-val is deprecated.  Use BagFromImages-msg:cols instead.")
  (cols m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Descriptor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BagFromImages-msg:type-val is deprecated.  Use BagFromImages-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'descdata-val :lambda-list '(m))
(cl:defmethod descdata-val ((m <Descriptor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader BagFromImages-msg:descdata-val is deprecated.  Use BagFromImages-msg:descdata instead.")
  (descdata m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Descriptor>) ostream)
  "Serializes a message object of type '<Descriptor>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cols)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cols)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cols)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cols)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'descdata))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'descdata))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Descriptor>) istream)
  "Deserializes a message object of type '<Descriptor>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cols)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cols)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cols)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cols)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'descdata) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'descdata)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Descriptor>)))
  "Returns string type for a message object of type '<Descriptor>"
  "BagFromImages/Descriptor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Descriptor)))
  "Returns string type for a message object of type 'Descriptor"
  "BagFromImages/Descriptor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Descriptor>)))
  "Returns md5sum for a message object of type '<Descriptor>"
  "e1da0f4e4634ed0f924a946dd2fbf324")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Descriptor)))
  "Returns md5sum for a message object of type 'Descriptor"
  "e1da0f4e4634ed0f924a946dd2fbf324")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Descriptor>)))
  "Returns full string definition for message of type '<Descriptor>"
  (cl:format cl:nil "~%Header header~%~%# OpenCV matrix containing the user data. A matrix of type CV_8UC1 ~%# with 1 row is considered to be compressed (with rtabmap::compressData() method).~%# If you have one dimension unsigned 8 bits uncompressed data, make sure to transpose it~%# (to have multiple rows instead of multiple columns) in order to be detected as~%# not compressed.~%uint32 rows~%uint32 cols~%uint32 type~%uint8[] descdata~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Descriptor)))
  "Returns full string definition for message of type 'Descriptor"
  (cl:format cl:nil "~%Header header~%~%# OpenCV matrix containing the user data. A matrix of type CV_8UC1 ~%# with 1 row is considered to be compressed (with rtabmap::compressData() method).~%# If you have one dimension unsigned 8 bits uncompressed data, make sure to transpose it~%# (to have multiple rows instead of multiple columns) in order to be detected as~%# not compressed.~%uint32 rows~%uint32 cols~%uint32 type~%uint8[] descdata~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Descriptor>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'descdata) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Descriptor>))
  "Converts a ROS message object to a list"
  (cl:list 'Descriptor
    (cl:cons ':header (header msg))
    (cl:cons ':rows (rows msg))
    (cl:cons ':cols (cols msg))
    (cl:cons ':type (type msg))
    (cl:cons ':descdata (descdata msg))
))
