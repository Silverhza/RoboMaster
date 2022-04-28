// Auto-generated. Do not edit!

// (in-package autofire.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Lidar2EnemyRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.setStatus = null;
    }
    else {
      if (initObj.hasOwnProperty('setStatus')) {
        this.setStatus = initObj.setStatus
      }
      else {
        this.setStatus = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lidar2EnemyRequest
    // Serialize message field [setStatus]
    bufferOffset = _serializer.bool(obj.setStatus, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lidar2EnemyRequest
    let len;
    let data = new Lidar2EnemyRequest(null);
    // Deserialize message field [setStatus]
    data.setStatus = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'autofire/Lidar2EnemyRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cba94cb4a8b514e916b961ea5660ffd7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool setStatus
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lidar2EnemyRequest(null);
    if (msg.setStatus !== undefined) {
      resolved.setStatus = msg.setStatus;
    }
    else {
      resolved.setStatus = false
    }

    return resolved;
    }
};

class Lidar2EnemyResponse {
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
    // Serializes a message object of type Lidar2EnemyResponse
    // Serialize message field [received]
    bufferOffset = _serializer.bool(obj.received, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lidar2EnemyResponse
    let len;
    let data = new Lidar2EnemyResponse(null);
    // Deserialize message field [received]
    data.received = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'autofire/Lidar2EnemyResponse';
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
    const resolved = new Lidar2EnemyResponse(null);
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
  Request: Lidar2EnemyRequest,
  Response: Lidar2EnemyResponse,
  md5sum() { return '5c20fa677eb90ce8ed6acc054aa17bb1'; },
  datatype() { return 'autofire/Lidar2Enemy'; }
};
