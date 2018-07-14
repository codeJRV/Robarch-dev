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

class MatrixFloat64 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.column_major = null;
      this.rows = null;
      this.cols = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('column_major')) {
        this.column_major = initObj.column_major
      }
      else {
        this.column_major = false;
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
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MatrixFloat64
    // Serialize message field [column_major]
    bufferOffset = _serializer.bool(obj.column_major, buffer, bufferOffset);
    // Serialize message field [rows]
    bufferOffset = _serializer.uint32(obj.rows, buffer, bufferOffset);
    // Serialize message field [cols]
    bufferOffset = _serializer.uint32(obj.cols, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.float64(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MatrixFloat64
    let len;
    let data = new MatrixFloat64(null);
    // Deserialize message field [column_major]
    data.column_major = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rows]
    data.rows = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cols]
    data.cols = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.data.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argus_msgs/MatrixFloat64';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd49ad88054ad35f5431113eea16faa1f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Double-precision dynamic-sized matrix message type
    bool column_major
    uint32 rows
    uint32 cols
    float64[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MatrixFloat64(null);
    if (msg.column_major !== undefined) {
      resolved.column_major = msg.column_major;
    }
    else {
      resolved.column_major = false
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

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = MatrixFloat64;
