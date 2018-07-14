// Auto-generated. Do not edit!

// (in-package argus_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FilterPredictStep = require('./FilterPredictStep.js');
let FilterUpdateStep = require('./FilterUpdateStep.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FilterStepInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.step_num = null;
      this.info_type = null;
      this.predict = null;
      this.update = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('step_num')) {
        this.step_num = initObj.step_num
      }
      else {
        this.step_num = 0;
      }
      if (initObj.hasOwnProperty('info_type')) {
        this.info_type = initObj.info_type
      }
      else {
        this.info_type = 0;
      }
      if (initObj.hasOwnProperty('predict')) {
        this.predict = initObj.predict
      }
      else {
        this.predict = new FilterPredictStep();
      }
      if (initObj.hasOwnProperty('update')) {
        this.update = initObj.update
      }
      else {
        this.update = new FilterUpdateStep();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FilterStepInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [step_num]
    bufferOffset = _serializer.uint64(obj.step_num, buffer, bufferOffset);
    // Serialize message field [info_type]
    bufferOffset = _serializer.uint8(obj.info_type, buffer, bufferOffset);
    // Serialize message field [predict]
    bufferOffset = FilterPredictStep.serialize(obj.predict, buffer, bufferOffset);
    // Serialize message field [update]
    bufferOffset = FilterUpdateStep.serialize(obj.update, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FilterStepInfo
    let len;
    let data = new FilterStepInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [step_num]
    data.step_num = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [info_type]
    data.info_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [predict]
    data.predict = FilterPredictStep.deserialize(buffer, bufferOffset);
    // Deserialize message field [update]
    data.update = FilterUpdateStep.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += FilterPredictStep.getMessageSize(object.predict);
    length += FilterUpdateStep.getMessageSize(object.update);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argus_msgs/FilterStepInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7f102f1a0b841c1cdbe4dbe82e22681e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message detailing a filter predict or update step
    #
    # Fields
    # ======
    # header/stamp     : Time at which this step ends or occurs at
    # header/frame_id  : This observation source's name [unused for predict]
    # step_num  : Filter step index
    # info_type : Enumerated active payload type
    # predict   : Predict info payload
    # update    : Update info payload
    
    # Defines the active payload field
    uint8 PREDICT_STEP = 0
    uint8 UPDATE_STEP = 1
    
    std_msgs/Header header
    uint64 step_num
    uint8 info_type
    argus_msgs/FilterPredictStep predict
    argus_msgs/FilterUpdateStep update
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: argus_msgs/FilterPredictStep
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
    ================================================================================
    MSG: argus_msgs/FilterUpdateStep
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FilterStepInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.step_num !== undefined) {
      resolved.step_num = msg.step_num;
    }
    else {
      resolved.step_num = 0
    }

    if (msg.info_type !== undefined) {
      resolved.info_type = msg.info_type;
    }
    else {
      resolved.info_type = 0
    }

    if (msg.predict !== undefined) {
      resolved.predict = FilterPredictStep.Resolve(msg.predict)
    }
    else {
      resolved.predict = new FilterPredictStep()
    }

    if (msg.update !== undefined) {
      resolved.update = FilterUpdateStep.Resolve(msg.update)
    }
    else {
      resolved.update = new FilterUpdateStep()
    }

    return resolved;
    }
};

// Constants for message
FilterStepInfo.Constants = {
  PREDICT_STEP: 0,
  UPDATE_STEP: 1,
}

module.exports = FilterStepInfo;
