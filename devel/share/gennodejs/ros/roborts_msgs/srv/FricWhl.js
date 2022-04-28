// Auto-generated. Do not edit!

// (in-package roborts_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class FricWhlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.open = null;
      this.left = null;
      this.right = null;
    }
    else {
      if (initObj.hasOwnProperty('open')) {
        this.open = initObj.open
      }
      else {
        this.open = false;
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = 0;
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FricWhlRequest
    // Serialize message field [open]
    bufferOffset = _serializer.bool(obj.open, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = _serializer.uint16(obj.left, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = _serializer.uint16(obj.right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FricWhlRequest
    let len;
    let data = new FricWhlRequest(null);
    // Deserialize message field [open]
    data.open = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roborts_msgs/FricWhlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fb182819872c1cdddaf2704fd8eb274d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool open
    uint16 left
    uint16 right
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FricWhlRequest(null);
    if (msg.open !== undefined) {
      resolved.open = msg.open;
    }
    else {
      resolved.open = false
    }

    if (msg.left !== undefined) {
      resolved.left = msg.left;
    }
    else {
      resolved.left = 0
    }

    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = 0
    }

    return resolved;
    }
};

class FricWhlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.received = null;
    }
    else {
      if (initObj.hasOwnProperty('received')) {
        this.received = initObj.received
      }
      else {
        this.received = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FricWhlResponse
    // Serialize message field [received]
    bufferOffset = _serializer.bool(obj.received, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FricWhlResponse
    let len;
    let data = new FricWhlResponse(null);
    // Deserialize message field [received]
    data.received = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'roborts_msgs/FricWhlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd4152e077925db952c78baadb1e48b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool received
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FricWhlResponse(null);
    if (msg.received !== undefined) {
      resolved.received = msg.received;
    }
    else {
      resolved.received = false
    }

    return resolved;
    }
};

module.exports = {
  Request: FricWhlRequest,
  Response: FricWhlResponse,
  md5sum() { return 'fa14985976e9b1bf0f4e4ece48d5775a'; },
  datatype() { return 'roborts_msgs/FricWhl'; }
};
