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

class ShootSwitchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.enableShoot = null;
    }
    else {
      if (initObj.hasOwnProperty('enableShoot')) {
        this.enableShoot = initObj.enableShoot
      }
      else {
        this.enableShoot = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ShootSwitchRequest
    // Serialize message field [enableShoot]
    bufferOffset = _serializer.bool(obj.enableShoot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ShootSwitchRequest
    let len;
    let data = new ShootSwitchRequest(null);
    // Deserialize message field [enableShoot]
    data.enableShoot = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'autofire/ShootSwitchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '375fc33df4063c841279eb3b6c0bb3e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool enableShoot
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ShootSwitchRequest(null);
    if (msg.enableShoot !== undefined) {
      resolved.enableShoot = msg.enableShoot;
    }
    else {
      resolved.enableShoot = false
    }

    return resolved;
    }
};

class ShootSwitchResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ShootSwitchResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ShootSwitchResponse
    let len;
    let data = new ShootSwitchResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'autofire/ShootSwitchResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a1255d4d998bd4d6585c64639b5ee9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ShootSwitchResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ShootSwitchRequest,
  Response: ShootSwitchResponse,
  md5sum() { return '5d377b7485c09603dc7fd050a7504601'; },
  datatype() { return 'autofire/ShootSwitch'; }
};
