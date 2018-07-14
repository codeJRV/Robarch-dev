; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude FilterStepInfo.msg.html

(cl:defclass <FilterStepInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (step_num
    :reader step_num
    :initarg :step_num
    :type cl:integer
    :initform 0)
   (info_type
    :reader info_type
    :initarg :info_type
    :type cl:fixnum
    :initform 0)
   (predict
    :reader predict
    :initarg :predict
    :type argus_msgs-msg:FilterPredictStep
    :initform (cl:make-instance 'argus_msgs-msg:FilterPredictStep))
   (update
    :reader update
    :initarg :update
    :type argus_msgs-msg:FilterUpdateStep
    :initform (cl:make-instance 'argus_msgs-msg:FilterUpdateStep)))
)

(cl:defclass FilterStepInfo (<FilterStepInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FilterStepInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FilterStepInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<FilterStepInfo> is deprecated: use argus_msgs-msg:FilterStepInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FilterStepInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:header-val is deprecated.  Use argus_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'step_num-val :lambda-list '(m))
(cl:defmethod step_num-val ((m <FilterStepInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:step_num-val is deprecated.  Use argus_msgs-msg:step_num instead.")
  (step_num m))

(cl:ensure-generic-function 'info_type-val :lambda-list '(m))
(cl:defmethod info_type-val ((m <FilterStepInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:info_type-val is deprecated.  Use argus_msgs-msg:info_type instead.")
  (info_type m))

(cl:ensure-generic-function 'predict-val :lambda-list '(m))
(cl:defmethod predict-val ((m <FilterStepInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:predict-val is deprecated.  Use argus_msgs-msg:predict instead.")
  (predict m))

(cl:ensure-generic-function 'update-val :lambda-list '(m))
(cl:defmethod update-val ((m <FilterStepInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:update-val is deprecated.  Use argus_msgs-msg:update instead.")
  (update m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FilterStepInfo>)))
    "Constants for message type '<FilterStepInfo>"
  '((:PREDICT_STEP . 0)
    (:UPDATE_STEP . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FilterStepInfo)))
    "Constants for message type 'FilterStepInfo"
  '((:PREDICT_STEP . 0)
    (:UPDATE_STEP . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FilterStepInfo>) ostream)
  "Serializes a message object of type '<FilterStepInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'step_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'info_type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'predict) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'update) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FilterStepInfo>) istream)
  "Deserializes a message object of type '<FilterStepInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'step_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'info_type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'predict) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'update) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FilterStepInfo>)))
  "Returns string type for a message object of type '<FilterStepInfo>"
  "argus_msgs/FilterStepInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FilterStepInfo)))
  "Returns string type for a message object of type 'FilterStepInfo"
  "argus_msgs/FilterStepInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FilterStepInfo>)))
  "Returns md5sum for a message object of type '<FilterStepInfo>"
  "7f102f1a0b841c1cdbe4dbe82e22681e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FilterStepInfo)))
  "Returns md5sum for a message object of type 'FilterStepInfo"
  "7f102f1a0b841c1cdbe4dbe82e22681e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FilterStepInfo>)))
  "Returns full string definition for message of type '<FilterStepInfo>"
  (cl:format cl:nil "# Message detailing a filter predict or update step~%#~%# Fields~%# ======~%# header/stamp     : Time at which this step ends or occurs at~%# header/frame_id  : This observation source's name [unused for predict]~%# step_num  : Filter step index~%# info_type : Enumerated active payload type~%# predict   : Predict info payload~%# update    : Update info payload~%~%# Defines the active payload field~%uint8 PREDICT_STEP = 0~%uint8 UPDATE_STEP = 1~%~%std_msgs/Header header~%uint64 step_num~%uint8 info_type~%argus_msgs/FilterPredictStep predict~%argus_msgs/FilterUpdateStep update~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: argus_msgs/FilterPredictStep~%# Message detailing a filter predict step~%#~%# Fields~%# ======~%# step_dt         : The predict time step size~%# trans_jacobian  : Transition function jacobian~%# trans_noise_cov : Transition noise covariance~%# prior_state_cov : State covariance before predict~%# post_state_cov  : State covariance after predict ~%~%float64 step_dt ~%MatrixFloat64 trans_jacobian~%MatrixFloat64 trans_noise_cov~%MatrixFloat64 prior_state_cov~%MatrixFloat64 post_state_cov~%================================================================================~%MSG: argus_msgs/MatrixFloat64~%# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%================================================================================~%MSG: argus_msgs/FilterUpdateStep~%# Message detailing a filter update step~%#~%# Fields~%# ======~%# prior_state_cov : State covariance before update~%# prior_obs_error : Observation prediction error before update~%# obs_error_cov   : Theoretical covariance of prediction error before update~%# post_state_cov  : State covariance after update~%# state_delta     : Correction applied to state from update~%# post_obs_error  : Observation prediction error after update~%# obs_jacobian    : Observation function jacobian~%# obs_noise_cov   : Observation noise covariance ~%~%MatrixFloat64 prior_state_cov~%float64[] prior_obs_error~%MatrixFloat64 obs_error_cov~%~%MatrixFloat64 post_state_cov~%float64[] state_delta~%float64[] post_obs_error~%~%MatrixFloat64 obs_jacobian~%MatrixFloat64 obs_noise_cov~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FilterStepInfo)))
  "Returns full string definition for message of type 'FilterStepInfo"
  (cl:format cl:nil "# Message detailing a filter predict or update step~%#~%# Fields~%# ======~%# header/stamp     : Time at which this step ends or occurs at~%# header/frame_id  : This observation source's name [unused for predict]~%# step_num  : Filter step index~%# info_type : Enumerated active payload type~%# predict   : Predict info payload~%# update    : Update info payload~%~%# Defines the active payload field~%uint8 PREDICT_STEP = 0~%uint8 UPDATE_STEP = 1~%~%std_msgs/Header header~%uint64 step_num~%uint8 info_type~%argus_msgs/FilterPredictStep predict~%argus_msgs/FilterUpdateStep update~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: argus_msgs/FilterPredictStep~%# Message detailing a filter predict step~%#~%# Fields~%# ======~%# step_dt         : The predict time step size~%# trans_jacobian  : Transition function jacobian~%# trans_noise_cov : Transition noise covariance~%# prior_state_cov : State covariance before predict~%# post_state_cov  : State covariance after predict ~%~%float64 step_dt ~%MatrixFloat64 trans_jacobian~%MatrixFloat64 trans_noise_cov~%MatrixFloat64 prior_state_cov~%MatrixFloat64 post_state_cov~%================================================================================~%MSG: argus_msgs/MatrixFloat64~%# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%================================================================================~%MSG: argus_msgs/FilterUpdateStep~%# Message detailing a filter update step~%#~%# Fields~%# ======~%# prior_state_cov : State covariance before update~%# prior_obs_error : Observation prediction error before update~%# obs_error_cov   : Theoretical covariance of prediction error before update~%# post_state_cov  : State covariance after update~%# state_delta     : Correction applied to state from update~%# post_obs_error  : Observation prediction error after update~%# obs_jacobian    : Observation function jacobian~%# obs_noise_cov   : Observation noise covariance ~%~%MatrixFloat64 prior_state_cov~%float64[] prior_obs_error~%MatrixFloat64 obs_error_cov~%~%MatrixFloat64 post_state_cov~%float64[] state_delta~%float64[] post_obs_error~%~%MatrixFloat64 obs_jacobian~%MatrixFloat64 obs_noise_cov~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FilterStepInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'predict))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'update))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FilterStepInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'FilterStepInfo
    (cl:cons ':header (header msg))
    (cl:cons ':step_num (step_num msg))
    (cl:cons ':info_type (info_type msg))
    (cl:cons ':predict (predict msg))
    (cl:cons ':update (update msg))
))
