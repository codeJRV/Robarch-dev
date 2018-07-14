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

class SymmetricFloat64 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dim = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('dim')) {
        this.dim = initObj.dim
      }
      else {
        this.dim = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SymmetricFloat64
    // Serialize message field [dim]
    bufferOffset = _serializer.uint32(obj.dim, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.float64(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SymmetricFloat64
    let len;
    let data = new SymmetricFloat64(null);
    // Deserialize message field [dim]
    data.dim = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.data.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argus_msgs/SymmetricFloat64';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '099dfa5da2b4d88d46f9e50c2fc822ad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Double-precision symmetric matrix message type
    uint32 dim # Rows = cols
    float64[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SymmetricFloat64(null);
    if (msg.dim !== undefined) {
      resolved.dim = msg.dim;
    }
    else {
      resolved.dim = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = SymmetricFloat64;
