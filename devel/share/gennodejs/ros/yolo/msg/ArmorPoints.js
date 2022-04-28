// Auto-generated. Do not edit!

// (in-package yolo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Box = require('./Box.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArmorPoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.numDetected = null;
      this.bbox = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('numDetected')) {
        this.numDetected = initObj.numDetected
      }
      else {
        this.numDetected = 0;
      }
      if (initObj.hasOwnProperty('bbox')) {
        this.bbox = initObj.bbox
      }
      else {
        this.bbox = new Array(6).fill(new Box());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmorPoints
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [numDetected]
    bufferOffset = _serializer.int32(obj.numDetected, buffer, bufferOffset);
    // Check that the constant length array field [bbox] has the right length
    if (obj.bbox.length !== 6) {
      throw new Error('Unable to serialize array field bbox - length must be 6')
    }
    // Serialize message field [bbox]
    obj.bbox.forEach((val) => {
      bufferOffset = Box.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmorPoints
    let len;
    let data = new ArmorPoints(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [numDetected]
    data.numDetected = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bbox]
    len = 6;
    data.bbox = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bbox[i] = Box.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.bbox.forEach((val) => {
      length += Box.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolo/ArmorPoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29bf3d3a8a44f27a0f27a1f9bc39e0c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 numDetected
    Box[6]  bbox
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
    MSG: yolo/Box
    float32 xmin # xmin
    float32 ymin  # ymin
    float32 xmax #xmax
    float32 ymax #ymax
    
    float32 confidence 
    string obj
    float32 distance
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmorPoints(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.numDetected !== undefined) {
      resolved.numDetected = msg.numDetected;
    }
    else {
      resolved.numDetected = 0
    }

    if (msg.bbox !== undefined) {
      resolved.bbox = new Array(6)
      for (let i = 0; i < resolved.bbox.length; ++i) {
        if (msg.bbox.length > i) {
          resolved.bbox[i] = Box.Resolve(msg.bbox[i]);
        }
        else {
          resolved.bbox[i] = new Box();
        }
      }
    }
    else {
      resolved.bbox = new Array(6).fill(new Box())
    }

    return resolved;
    }
};

module.exports = ArmorPoints;
