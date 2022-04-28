// Auto-generated. Do not edit!

// (in-package obstacle_detector.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SegmentObstacle = require('./SegmentObstacle.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class CircleObstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.center = null;
      this.velocity = null;
      this.radius = null;
      this.true_radius = null;
      this.segment = null;
    }
    else {
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('true_radius')) {
        this.true_radius = initObj.true_radius
      }
      else {
        this.true_radius = 0.0;
      }
      if (initObj.hasOwnProperty('segment')) {
        this.segment = initObj.segment
      }
      else {
        this.segment = new SegmentObstacle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CircleObstacle
    // Serialize message field [center]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [true_radius]
    bufferOffset = _serializer.float64(obj.true_radius, buffer, bufferOffset);
    // Serialize message field [segment]
    bufferOffset = SegmentObstacle.serialize(obj.segment, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CircleObstacle
    let len;
    let data = new CircleObstacle(null);
    // Deserialize message field [center]
    data.center = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [true_radius]
    data.true_radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [segment]
    data.segment = SegmentObstacle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'obstacle_detector/CircleObstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0b960765a2e073d18b58abefb071095b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point center      # Central point [m]
    geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
    float64 radius                  # Radius with added margin [m]
    float64 true_radius             # True measured radius [m]
    obstacle_detector/SegmentObstacle segment    # Keep track of the segment which the circle was spawned
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: obstacle_detector/SegmentObstacle
    geometry_msgs/Point first_point  # First point of the segment [m]
    geometry_msgs/Point last_point   # Last point of the segment [m]
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CircleObstacle(null);
    if (msg.center !== undefined) {
      resolved.center = geometry_msgs.msg.Point.Resolve(msg.center)
    }
    else {
      resolved.center = new geometry_msgs.msg.Point()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.true_radius !== undefined) {
      resolved.true_radius = msg.true_radius;
    }
    else {
      resolved.true_radius = 0.0
    }

    if (msg.segment !== undefined) {
      resolved.segment = SegmentObstacle.Resolve(msg.segment)
    }
    else {
      resolved.segment = new SegmentObstacle()
    }

    return resolved;
    }
};

module.exports = CircleObstacle;
