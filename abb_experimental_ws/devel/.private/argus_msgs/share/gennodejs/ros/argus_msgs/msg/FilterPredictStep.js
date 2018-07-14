// Auto-generated. Do not edit!

// (in-package argus_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MatrixFloat64 = require('./MatrixFloat64.js');

//-----------------------------------------------------------

class FilterPredictStep {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.step_dt = null;
      this.trans_jacobian = null;
      this.trans_noise_cov = null;
      this.prior_state_cov = null;
      this.post_state_cov = null;
    }
    else {
      if (initObj.hasOwnProperty('step_dt')) {
        this.step_dt = initObj.step_dt
      }
      else {
        this.step_dt = 0.0;
      }
      if (initObj.hasOwnProperty('trans_jacobian')) {
        this.trans_jacobian = initObj.trans_jacobian
      }
      else {
        this.trans_jacobian = new MatrixFloat64();
      }
      if (initObj.hasOwnProperty('trans_noise_cov')) {
        this.trans_noise_cov = initObj.trans_noise_cov
      }
      else {
        this.trans_noise_cov = new MatrixFloat64();
      }
      if (initObj.hasOwnProperty('prior_state_cov')) {
        this.prior_state_cov = initObj.prior_state_cov
      }
      else {
        this.prior_state_cov = new MatrixFloat64();
      }
      if (initObj.hasOwnProperty('post_state_cov')) {
        this.post_state_cov = initObj.post_state_cov
      }
      else {
        this.post_state_cov = new MatrixFloat64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FilterPredictStep
    // Serialize message field [step_dt]
    bufferOffset = _serializer.float64(obj.step_dt, buffer, bufferOffset);
    // Serialize message field [trans_jacobian]
    bufferOffset = MatrixFloat64.serialize(obj.trans_jacobian, buffer, bufferOffset);
    // Serialize message field [trans_noise_cov]
    bufferOffset = MatrixFloat64.serialize(obj.trans_noise_cov, buffer, bufferOffset);
    // Serialize message field [prior_state_cov]
    bufferOffset = MatrixFloat64.serialize(obj.prior_state_cov, buffer, bufferOffset);
    // Serialize message field [post_state_cov]
    bufferOffset = MatrixFloat64.serialize(obj.post_state_cov, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FilterPredictStep
    let len;
    let data = new FilterPredictStep(null);
    // Deserialize message field [step_dt]
    data.step_dt = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [trans_jacobian]
    data.trans_jacobian = MatrixFloat64.deserialize(buffer, bufferOffset);
    // Deserialize message field [trans_noise_cov]
    data.trans_noise_cov = MatrixFloat64.deserialize(buffer, bufferOffset);
    // Deserialize message field [prior_state_cov]
    data.prior_state_cov = MatrixFloat64.deserialize(buffer, bufferOffset);
    // Deserialize message field [post_state_cov]
    data.post_state_cov = MatrixFloat64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += MatrixFloat64.getMessageSize(object.trans_jacobian);
    length += MatrixFloat64.getMessageSize(object.trans_noise_cov);
    length += MatrixFloat64.getMessageSize(object.prior_state_cov);
    length += MatrixFloat64.getMessageSize(object.post_state_cov);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argus_msgs/FilterPredictStep';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b900620feacd2fcbfc52c9eef367d6d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message detailing a filter predict step
    #
    # Fields
    # ======
    # step_dt         : The predict time step size
    # trans_jacobian  : Transition function jacobian
    # trans_noise_cov : Transition noise covariance
    # prior_state_cov : State covariance before predict
    # post_state_cov  : State covariance after predict 
    
    float64 step_dt 
    MatrixFloat64 trans_jacobian
    MatrixFloat64 trans_noise_cov
    MatrixFloat64 prior_state_cov
    MatrixFloat64 post_state_cov
    ================================================================================
    MSG: argus_msgs/MatrixFloat64
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
    const resolved = new FilterPredictStep(null);
    if (msg.step_dt !== undefined) {
      resolved.step_dt = msg.step_dt;
    }
    else {
      resolved.step_dt = 0.0
    }

    if (msg.trans_jacobian !== undefined) {
      resolved.trans_jacobian = MatrixFloat64.Resolve(msg.trans_jacobian)
    }
    else {
      resolved.trans_jacobian = new MatrixFloat64()
    }

    if (msg.trans_noise_cov !== undefined) {
      resolved.trans_noise_cov = MatrixFloat64.Resolve(msg.trans_noise_cov)
    }
    else {
      resolved.trans_noise_cov = new MatrixFloat64()
    }

    if (msg.prior_state_cov !== undefined) {
      resolved.prior_state_cov = MatrixFloat64.Resolve(msg.prior_state_cov)
    }
    else {
      resolved.prior_state_cov = new MatrixFloat64()
    }

    if (msg.post_state_cov !== undefined) {
      resolved.post_state_cov = MatrixFloat64.Resolve(msg.post_state_cov)
    }
    else {
      resolved.post_state_cov = new MatrixFloat64()
    }

    return resolved;
    }
};

module.exports = FilterPredictStep;
