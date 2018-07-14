// Auto-generated. Do not edit!

// (in-package argus_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class EstimatePerformance {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rms_linear_vel = null;
      this.rms_angular_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('rms_linear_vel')) {
        this.rms_linear_vel = initObj.rms_linear_vel
      }
      else {
        this.rms_linear_vel = 0.0;
      }
      if (initObj.hasOwnProperty('rms_angular_vel')) {
        this.rms_angular_vel = initObj.rms_angular_vel
      }
      else {
        this.rms_angular_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EstimatePerformance
    // Serialize message field [rms_linear_vel]
    bufferOffset = _serializer.float64(obj.rms_linear_vel, buffer, bufferOffset);
    // Serialize message field [rms_angular_vel]
    bufferOffset = _serializer.float64(obj.rms_angular_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EstimatePerformance
    let len;
    let data = new EstimatePerformance(null);
    // Deserialize message field [rms_linear_vel]
    data.rms_linear_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rms_angular_vel]
    data.rms_angular_vel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argus_msgs/EstimatePerformance';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17e5293affcaac44e00d9f001150b5b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message with state estimate performance
    
    # Root mean square error for linear velocity
    float64 rms_linear_vel
    
    # Root mean square error for angular velocity
    float64 rms_angular_vel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EstimatePerformance(null);
    if (msg.rms_linear_vel !== undefined) {
      resolved.rms_linear_vel = msg.rms_linear_vel;
    }
    else {
      resolved.rms_linear_vel = 0.0
    }

    if (msg.rms_angular_vel !== undefined) {
      resolved.rms_angular_vel = msg.rms_angular_vel;
    }
    else {
      resolved.rms_angular_vel = 0.0
    }

    return resolved;
    }
};

module.exports = EstimatePerformance;
