// Auto-generated. Do not edit!

// (in-package BagFromImages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let KeyPoint = require('./KeyPoint.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class KeyPoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.keypoints = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('keypoints')) {
        this.keypoints = initObj.keypoints
      }
      else {
        this.keypoints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KeyPoints
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [keypoints]
    // Serialize the length for message field [keypoints]
    bufferOffset = _serializer.uint32(obj.keypoints.length, buffer, bufferOffset);
    obj.keypoints.forEach((val) => {
      bufferOffset = KeyPoint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KeyPoints
    let len;
    let data = new KeyPoints(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [keypoints]
    // Deserialize array length for message field [keypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.keypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.keypoints[i] = KeyPoint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 28 * object.keypoints.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'BagFromImages/KeyPoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ada03370f65713b6c35c9e1949b83815';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KeyPoints(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.keypoints !== undefined) {
      resolved.keypoints = new Array(msg.keypoints.length);
      for (let i = 0; i < resolved.keypoints.length; ++i) {
        resolved.keypoints[i] = KeyPoint.Resolve(msg.keypoints[i]);
      }
    }
    else {
      resolved.keypoints = []
    }

    return resolved;
    }
};

module.exports = KeyPoints;
