// Auto-generated. Do not edit!

// (in-package orb_slam3_ros_wrapper.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Descriptor {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.rows = null;
      this.cols = null;
      this.type = null;
      this.descdata = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('rows')) {
        this.rows = initObj.rows
      }
      else {
        this.rows = 0;
      }
      if (initObj.hasOwnProperty('cols')) {
        this.cols = initObj.cols
      }
      else {
        this.cols = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('descdata')) {
        this.descdata = initObj.descdata
      }
      else {
        this.descdata = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Descriptor
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [rows]
    bufferOffset = _serializer.uint32(obj.rows, buffer, bufferOffset);
    // Serialize message field [cols]
    bufferOffset = _serializer.uint32(obj.cols, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint32(obj.type, buffer, bufferOffset);
    // Serialize message field [descdata]
    bufferOffset = _arraySerializer.uint8(obj.descdata, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Descriptor
    let len;
    let data = new Descriptor(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [rows]
    data.rows = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cols]
    data.cols = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [descdata]
    data.descdata = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.descdata.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'orb_slam3_ros_wrapper/Descriptor';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1da0f4e4634ed0f924a946dd2fbf324';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Descriptor(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.rows !== undefined) {
      resolved.rows = msg.rows;
    }
    else {
      resolved.rows = 0
    }

    if (msg.cols !== undefined) {
      resolved.cols = msg.cols;
    }
    else {
      resolved.cols = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.descdata !== undefined) {
      resolved.descdata = msg.descdata;
    }
    else {
      resolved.descdata = []
    }

    return resolved;
    }
};

module.exports = Descriptor;
