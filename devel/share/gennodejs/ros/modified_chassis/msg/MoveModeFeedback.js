// Auto-generated. Do not edit!

// (in-package modified_chassis.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MoveModeFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state_keeping = null;
    }
    else {
      if (initObj.hasOwnProperty('state_keeping')) {
        this.state_keeping = initObj.state_keeping
      }
      else {
        this.state_keeping = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveModeFeedback
    // Serialize message field [state_keeping]
    bufferOffset = _serializer.float32(obj.state_keeping, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveModeFeedback
    let len;
    let data = new MoveModeFeedback(null);
    // Deserialize message field [state_keeping]
    data.state_keeping = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'modified_chassis/MoveModeFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f643aa8d8b0e9b88fb16c443fac7ea19';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # Define a feedback message
    float32 state_keeping
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveModeFeedback(null);
    if (msg.state_keeping !== undefined) {
      resolved.state_keeping = msg.state_keeping;
    }
    else {
      resolved.state_keeping = 0.0
    }

    return resolved;
    }
};

module.exports = MoveModeFeedback;
