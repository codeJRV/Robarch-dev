; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude FilterPredictStep.msg.html

(cl:defclass <FilterPredictStep> (roslisp-msg-protocol:ros-message)
  ((step_dt
    :reader step_dt
    :initarg :step_dt
    :type cl:float
    :initform 0.0)
   (trans_jacobian
    :reader trans_jacobian
    :initarg :trans_jacobian
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64))
   (trans_noise_cov
    :reader trans_noise_cov
    :initarg :trans_noise_cov
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64))
   (prior_state_cov
    :reader prior_state_cov
    :initarg :prior_state_cov
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64))
   (post_state_cov
    :reader post_state_cov
    :initarg :post_state_cov
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64)))
)

(cl:defclass FilterPredictStep (<FilterPredictStep>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FilterPredictStep>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FilterPredictStep)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<FilterPredictStep> is deprecated: use argus_msgs-msg:FilterPredictStep instead.")))

(cl:ensure-generic-function 'step_dt-val :lambda-list '(m))
(cl:defmethod step_dt-val ((m <FilterPredictStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:step_dt-val is deprecated.  Use argus_msgs-msg:step_dt instead.")
  (step_dt m))

(cl:ensure-generic-function 'trans_jacobian-val :lambda-list '(m))
(cl:defmethod trans_jacobian-val ((m <FilterPredictStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:trans_jacobian-val is deprecated.  Use argus_msgs-msg:trans_jacobian instead.")
  (trans_jacobian m))

(cl:ensure-generic-function 'trans_noise_cov-val :lambda-list '(m))
(cl:defmethod trans_noise_cov-val ((m <FilterPredictStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:trans_noise_cov-val is deprecated.  Use argus_msgs-msg:trans_noise_cov instead.")
  (trans_noise_cov m))

(cl:ensure-generic-function 'prior_state_cov-val :lambda-list '(m))
(cl:defmethod prior_state_cov-val ((m <FilterPredictStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:prior_state_cov-val is deprecated.  Use argus_msgs-msg:prior_state_cov instead.")
  (prior_state_cov m))

(cl:ensure-generic-function 'post_state_cov-val :lambda-list '(m))
(cl:defmethod post_state_cov-val ((m <FilterPredictStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:post_state_cov-val is deprecated.  Use argus_msgs-msg:post_state_cov instead.")
  (post_state_cov m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FilterPredictStep>) ostream)
  "Serializes a message object of type '<FilterPredictStep>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'step_dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trans_jacobian) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trans_noise_cov) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'prior_state_cov) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'post_state_cov) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FilterPredictStep>) istream)
  "Deserializes a message object of type '<FilterPredictStep>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'step_dt) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trans_jacobian) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trans_noise_cov) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'prior_state_cov) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'post_state_cov) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FilterPredictStep>)))
  "Returns string type for a message object of type '<FilterPredictStep>"
  "argus_msgs/FilterPredictStep")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FilterPredictStep)))
  "Returns string type for a message object of type 'FilterPredictStep"
  "argus_msgs/FilterPredictStep")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FilterPredictStep>)))
  "Returns md5sum for a message object of type '<FilterPredictStep>"
  "b900620feacd2fcbfc52c9eef367d6d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FilterPredictStep)))
  "Returns md5sum for a message object of type 'FilterPredictStep"
  "b900620feacd2fcbfc52c9eef367d6d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FilterPredictStep>)))
  "Returns full string definition for message of type '<FilterPredictStep>"
  (cl:format cl:nil "# Message detailing a filter predict step~%#~%# Fields~%# ======~%# step_dt         : The predict time step size~%# trans_jacobian  : Transition function jacobian~%# trans_noise_cov : Transition noise covariance~%# prior_state_cov : State covariance before predict~%# post_state_cov  : State covariance after predict ~%~%float64 step_dt ~%MatrixFloat64 trans_jacobian~%MatrixFloat64 trans_noise_cov~%MatrixFloat64 prior_state_cov~%MatrixFloat64 post_state_cov~%================================================================================~%MSG: argus_msgs/MatrixFloat64~%# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FilterPredictStep)))
  "Returns full string definition for message of type 'FilterPredictStep"
  (cl:format cl:nil "# Message detailing a filter predict step~%#~%# Fields~%# ======~%# step_dt         : The predict time step size~%# trans_jacobian  : Transition function jacobian~%# trans_noise_cov : Transition noise covariance~%# prior_state_cov : State covariance before predict~%# post_state_cov  : State covariance after predict ~%~%float64 step_dt ~%MatrixFloat64 trans_jacobian~%MatrixFloat64 trans_noise_cov~%MatrixFloat64 prior_state_cov~%MatrixFloat64 post_state_cov~%================================================================================~%MSG: argus_msgs/MatrixFloat64~%# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FilterPredictStep>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trans_jacobian))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trans_noise_cov))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'prior_state_cov))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'post_state_cov))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FilterPredictStep>))
  "Converts a ROS message object to a list"
  (cl:list 'FilterPredictStep
    (cl:cons ':step_dt (step_dt msg))
    (cl:cons ':trans_jacobian (trans_jacobian msg))
    (cl:cons ':trans_noise_cov (trans_noise_cov msg))
    (cl:cons ':prior_state_cov (prior_state_cov msg))
    (cl:cons ':post_state_cov (post_state_cov msg))
))
