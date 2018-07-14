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

class FilterUpdateStep {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prior_state_cov = null;
      this.prior_obs_error = null;
      this.obs_error_cov = null;
      this.post_state_cov = null;
      this.state_delta = null;
      this.post_obs_error = null;
      this.obs_jacobian = null;
      this.obs_noise_cov = null;
    }
    else {
      if (initObj.hasOwnProperty('prior_state_cov')) {
        this.prior_state_cov = initObj.prior_state_cov
      }
      else {
        this.prior_state_cov = new MatrixFloat64();
      }
      if (initObj.hasOwnProperty('prior_obs_error')) {
        this.prior_obs_error = initObj.prior_obs_error
      }
      else {
        this.prior_obs_error = [];
      }
      if (initObj.hasOwnProperty('obs_error_cov')) {
        this.obs_error_cov = initObj.obs_error_cov
      }
      else {
        this.obs_error_cov = new MatrixFloat64();
      }
      if (initObj.hasOwnProperty('post_state_cov')) {
        this.post_state_cov = initObj.post_state_cov
      }
      else {
        this.post_state_cov = new MatrixFloat64();
      }
      if (initObj.hasOwnProperty('state_delta')) {
        this.state_delta = initObj.state_delta
      }
      else {
        this.state_delta = [];
      }
      if (initObj.hasOwnProperty('post_obs_error')) {
        this.post_obs_error = initObj.post_obs_error
      }
      else {
        this.post_obs_error = [];
      }
      if (initObj.hasOwnProperty('obs_jacobian')) {
        this.obs_jacobian = initObj.obs_jacobian
      }
      else {
        this.obs_jacobian = new MatrixFloat64();
      }
      if (initObj.hasOwnProperty('obs_noise_cov')) {
        this.obs_noise_cov = initObj.obs_noise_cov
      }
      else {
        this.obs_noise_cov = new MatrixFloat64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FilterUpdateStep
    // Serialize message field [prior_state_cov]
    bufferOffset = MatrixFloat64.serialize(obj.prior_state_cov, buffer, bufferOffset);
    // Serialize message field [prior_obs_error]
    bufferOffset = _arraySerializer.float64(obj.prior_obs_error, buffer, bufferOffset, null);
    // Serialize message field [obs_error_cov]
    bufferOffset = MatrixFloat64.serialize(obj.obs_error_cov, buffer, bufferOffset);
    // Serialize message field [post_state_cov]
    bufferOffset = MatrixFloat64.serialize(obj.post_state_cov, buffer, bufferOffset);
    // Serialize message field [state_delta]
    bufferOffset = _arraySerializer.float64(obj.state_delta, buffer, bufferOffset, null);
    // Serialize message field [post_obs_error]
    bufferOffset = _arraySerializer.float64(obj.post_obs_error, buffer, bufferOffset, null);
    // Serialize message field [obs_jacobian]
    bufferOffset = MatrixFloat64.serialize(obj.obs_jacobian, buffer, bufferOffset);
    // Serialize message field [obs_noise_cov]
    bufferOffset = MatrixFloat64.serialize(obj.obs_noise_cov, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FilterUpdateStep
    let len;
    let data = new FilterUpdateStep(null);
    // Deserialize message field [prior_state_cov]
    data.prior_state_cov = MatrixFloat64.deserialize(buffer, bufferOffset);
    // Deserialize message field [prior_obs_error]
    data.prior_obs_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [obs_error_cov]
    data.obs_error_cov = MatrixFloat64.deserialize(buffer, bufferOffset);
    // Deserialize message field [post_state_cov]
    data.post_state_cov = MatrixFloat64.deserialize(buffer, bufferOffset);
    // Deserialize message field [state_delta]
    data.state_delta = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [post_obs_error]
    data.post_obs_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [obs_jacobian]
    data.obs_jacobian = MatrixFloat64.deserialize(buffer, bufferOffset);
    // Deserialize message field [obs_noise_cov]
    data.obs_noise_cov = MatrixFloat64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += MatrixFloat64.getMessageSize(object.prior_state_cov);
    length += 8 * object.prior_obs_error.length;
    length += MatrixFloat64.getMessageSize(object.obs_error_cov);
    length += MatrixFloat64.getMessageSize(object.post_state_cov);
    length += 8 * object.state_delta.length;
    length += 8 * object.post_obs_error.length;
    length += MatrixFloat64.getMessageSize(object.obs_jacobian);
    length += MatrixFloat64.getMessageSize(object.obs_noise_cov);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argus_msgs/FilterUpdateStep';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '451710d77b207f2e2068272b8ce64023';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message detailing a filter update step
    #
    # Fields
    # ======
    # prior_state_cov : State covariance before update
    # prior_obs_error : Observation prediction error before update
    # obs_error_cov   : Theoretical covariance of prediction error before update
    # post_state_cov  : State covariance after update
    # state_delta     : Correction applied to state from update
    # post_obs_error  : Observation prediction error after update
    # obs_jacobian    : Observation function jacobian
    # obs_noise_cov   : Observation noise covariance 
    
    MatrixFloat64 prior_state_cov
    float64[] prior_obs_error
    MatrixFloat64 obs_error_cov
    
    MatrixFloat64 post_state_cov
    float64[] state_delta
    float64[] post_obs_error
    
    MatrixFloat64 obs_jacobian
    MatrixFloat64 obs_noise_cov
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
    const resolved = new FilterUpdateStep(null);
    if (msg.prior_state_cov !== undefined) {
      resolved.prior_state_cov = MatrixFloat64.Resolve(msg.prior_state_cov)
    }
    else {
      resolved.prior_state_cov = new MatrixFloat64()
    }

    if (msg.prior_obs_error !== undefined) {
      resolved.prior_obs_error = msg.prior_obs_error;
    }
    else {
      resolved.prior_obs_error = []
    }

    if (msg.obs_error_cov !== undefined) {
      resolved.obs_error_cov = MatrixFloat64.Resolve(msg.obs_error_cov)
    }
    else {
      resolved.obs_error_cov = new MatrixFloat64()
    }

    if (msg.post_state_cov !== undefined) {
      resolved.post_state_cov = MatrixFloat64.Resolve(msg.post_state_cov)
    }
    else {
      resolved.post_state_cov = new MatrixFloat64()
    }

    if (msg.state_delta !== undefined) {
      resolved.state_delta = msg.state_delta;
    }
    else {
      resolved.state_delta = []
    }

    if (msg.post_obs_error !== undefined) {
      resolved.post_obs_error = msg.post_obs_error;
    }
    else {
      resolved.post_obs_error = []
    }

    if (msg.obs_jacobian !== undefined) {
      resolved.obs_jacobian = MatrixFloat64.Resolve(msg.obs_jacobian)
    }
    else {
      resolved.obs_jacobian = new MatrixFloat64()
    }

    if (msg.obs_noise_cov !== undefined) {
      resolved.obs_noise_cov = MatrixFloat64.Resolve(msg.obs_noise_cov)
    }
    else {
      resolved.obs_noise_cov = new MatrixFloat64()
    }

    return resolved;
    }
};

module.exports = FilterUpdateStep;
