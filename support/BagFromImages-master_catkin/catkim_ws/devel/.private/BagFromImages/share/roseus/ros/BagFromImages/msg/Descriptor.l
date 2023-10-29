;; Auto-generated. Do not edit!


(when (boundp 'BagFromImages::Descriptor)
  (if (not (find-package "BAGFROMIMAGES"))
    (make-package "BAGFROMIMAGES"))
  (shadow 'Descriptor (find-package "BAGFROMIMAGES")))
(unless (find-package "BAGFROMIMAGES::DESCRIPTOR")
  (make-package "BAGFROMIMAGES::DESCRIPTOR"))

(in-package "ROS")
;;//! \htmlinclude Descriptor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass BagFromImages::Descriptor
  :super ros::object
  :slots (_header _rows _cols _type _descdata ))

(defmethod BagFromImages::Descriptor
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rows __rows) 0)
    ((:cols __cols) 0)
    ((:type __type) 0)
    ((:descdata __descdata) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _rows (round __rows))
   (setq _cols (round __cols))
   (setq _type (round __type))
   (setq _descdata __descdata)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rows
   (&optional __rows)
   (if __rows (setq _rows __rows)) _rows)
  (:cols
   (&optional __cols)
   (if __cols (setq _cols __cols)) _cols)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:descdata
   (&optional __descdata)
   (if __descdata (setq _descdata __descdata)) _descdata)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _rows
    4
    ;; uint32 _cols
    4
    ;; uint32 _type
    4
    ;; uint8[] _descdata
    (* 1    (length _descdata)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _rows
       (write-long _rows s)
     ;; uint32 _cols
       (write-long _cols s)
     ;; uint32 _type
       (write-long _type s)
     ;; uint8[] _descdata
     (write-long (length _descdata) s)
     (princ _descdata s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _rows
     (setq _rows (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _cols
     (setq _cols (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _descdata
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _descdata (make-array n :element-type :char))
     (replace _descdata buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get BagFromImages::Descriptor :md5sum-) "e1da0f4e4634ed0f924a946dd2fbf324")
(setf (get BagFromImages::Descriptor :datatype-) "BagFromImages/Descriptor")
(setf (get BagFromImages::Descriptor :definition-)
      "
Header header

# OpenCV matrix containing the user data. A matrix of type CV_8UC1 
# with 1 row is considered to be compressed (with rtabmap::compressData() method).
# If you have one dimension unsigned 8 bits uncompressed data, make sure to transpose it
# (to have multiple rows instead of multiple columns) in order to be detected as
# not compressed.
uint32 rows
uint32 cols
uint32 type
uint8[] descdata
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



(provide :BagFromImages/Descriptor "e1da0f4e4634ed0f924a946dd2fbf324")


