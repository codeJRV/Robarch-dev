// Auto-generated. Do not edit!

// (in-package argus_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Point2D = require('./Point2D.js');

//-----------------------------------------------------------

class FiducialDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.undistorted = null;
      this.normalized = null;
      this.points = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('undistorted')) {
        this.undistorted = initObj.undistorted
      }
      else {
        this.undistorted = false;
      }
      if (initObj.hasOwnProperty('normalized')) {
        this.normalized = initObj.normalized
      }
      else {
        this.normalized = false;
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FiducialDetection
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [undistorted]
    bufferOffset = _serializer.bool(obj.undistorted, buffer, bufferOffset);
    // Serialize message field [normalized]
    bufferOffset = _serializer.bool(obj.normalized, buffer, bufferOffset);
    // Serialize message field [points]
    // Serialize the length for message field [points]
    bufferOffset = _serializer.uint32(obj.points.length, buffer, bufferOffset);
    obj.points.forEach((val) => {
      bufferOffset = Point2D.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FiducialDetection
    let len;
    let data = new FiducialDetection(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [undistorted]
    data.undistorted = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [normalized]
    data.normalized = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [points]
    // Deserialize array length for message field [points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points[i] = Point2D.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    length += 16 * object.points.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argus_msgs/FiducialDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9798b72f528ec3822efac6e4bf3400a7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The name of this fiducial
    string name
    
    # Whether the points are undistorted
    bool undistorted
    
    # Whether the points are in normalized image coordinates
    bool normalized
    
    # The detected points
    Point2D[] points
    
    ================================================================================
    MSG: argus_msgs/Point2D
    float64 x
    float64 y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FiducialDetection(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.undistorted !== undefined) {
      resolved.undistorted = msg.undistorted;
    }
    else {
      resolved.undistorted = false
    }

    if (msg.normalized !== undefined) {
      resolved.normalized = msg.normalized;
    }
    else {
      resolved.normalized = false
    }

    if (msg.points !== undefined) {
      resolved.points = new Array(msg.points.length);
      for (let i = 0; i < resolved.points.length; ++i) {
        resolved.points[i] = Point2D.Resolve(msg.points[i]);
      }
    }
    else {
      resolved.points = []
    }

    return resolved;
    }
};

module.exports = FiducialDetection;
