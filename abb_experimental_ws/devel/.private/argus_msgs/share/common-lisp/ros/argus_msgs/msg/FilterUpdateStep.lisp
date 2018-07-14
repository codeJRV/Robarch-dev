; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude FilterUpdateStep.msg.html

(cl:defclass <FilterUpdateStep> (roslisp-msg-protocol:ros-message)
  ((prior_state_cov
    :reader prior_state_cov
    :initarg :prior_state_cov
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64))
   (prior_obs_error
    :reader prior_obs_error
    :initarg :prior_obs_error
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (obs_error_cov
    :reader obs_error_cov
    :initarg :obs_error_cov
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64))
   (post_state_cov
    :reader post_state_cov
    :initarg :post_state_cov
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64))
   (state_delta
    :reader state_delta
    :initarg :state_delta
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (post_obs_error
    :reader post_obs_error
    :initarg :post_obs_error
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (obs_jacobian
    :reader obs_jacobian
    :initarg :obs_jacobian
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64))
   (obs_noise_cov
    :reader obs_noise_cov
    :initarg :obs_noise_cov
    :type argus_msgs-msg:MatrixFloat64
    :initform (cl:make-instance 'argus_msgs-msg:MatrixFloat64)))
)

(cl:defclass FilterUpdateStep (<FilterUpdateStep>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FilterUpdateStep>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FilterUpdateStep)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<FilterUpdateStep> is deprecated: use argus_msgs-msg:FilterUpdateStep instead.")))

(cl:ensure-generic-function 'prior_state_cov-val :lambda-list '(m))
(cl:defmethod prior_state_cov-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:prior_state_cov-val is deprecated.  Use argus_msgs-msg:prior_state_cov instead.")
  (prior_state_cov m))

(cl:ensure-generic-function 'prior_obs_error-val :lambda-list '(m))
(cl:defmethod prior_obs_error-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:prior_obs_error-val is deprecated.  Use argus_msgs-msg:prior_obs_error instead.")
  (prior_obs_error m))

(cl:ensure-generic-function 'obs_error_cov-val :lambda-list '(m))
(cl:defmethod obs_error_cov-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:obs_error_cov-val is deprecated.  Use argus_msgs-msg:obs_error_cov instead.")
  (obs_error_cov m))

(cl:ensure-generic-function 'post_state_cov-val :lambda-list '(m))
(cl:defmethod post_state_cov-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:post_state_cov-val is deprecated.  Use argus_msgs-msg:post_state_cov instead.")
  (post_state_cov m))

(cl:ensure-generic-function 'state_delta-val :lambda-list '(m))
(cl:defmethod state_delta-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:state_delta-val is deprecated.  Use argus_msgs-msg:state_delta instead.")
  (state_delta m))

(cl:ensure-generic-function 'post_obs_error-val :lambda-list '(m))
(cl:defmethod post_obs_error-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:post_obs_error-val is deprecated.  Use argus_msgs-msg:post_obs_error instead.")
  (post_obs_error m))

(cl:ensure-generic-function 'obs_jacobian-val :lambda-list '(m))
(cl:defmethod obs_jacobian-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:obs_jacobian-val is deprecated.  Use argus_msgs-msg:obs_jacobian instead.")
  (obs_jacobian m))

(cl:ensure-generic-function 'obs_noise_cov-val :lambda-list '(m))
(cl:defmethod obs_noise_cov-val ((m <FilterUpdateStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:obs_noise_cov-val is deprecated.  Use argus_msgs-msg:obs_noise_cov instead.")
  (obs_noise_cov m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FilterUpdateStep>) ostream)
  "Serializes a message object of type '<FilterUpdateStep>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'prior_state_cov) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'prior_obs_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'prior_obs_error))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obs_error_cov) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'post_state_cov) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'state_delta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'state_delta))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'post_obs_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'post_obs_error))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obs_jacobian) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obs_noise_cov) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FilterUpdateStep>) istream)
  "Deserializes a message object of type '<FilterUpdateStep>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'prior_state_cov) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'prior_obs_error) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'prior_obs_error)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obs_error_cov) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'post_state_cov) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'state_delta) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'state_delta)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'post_obs_error) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'post_obs_error)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obs_jacobian) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obs_noise_cov) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FilterUpdateStep>)))
  "Returns string type for a message object of type '<FilterUpdateStep>"
  "argus_msgs/FilterUpdateStep")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FilterUpdateStep)))
  "Returns string type for a message object of type 'FilterUpdateStep"
  "argus_msgs/FilterUpdateStep")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FilterUpdateStep>)))
  "Returns md5sum for a message object of type '<FilterUpdateStep>"
  "451710d77b207f2e2068272b8ce64023")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FilterUpdateStep)))
  "Returns md5sum for a message object of type 'FilterUpdateStep"
  "451710d77b207f2e2068272b8ce64023")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FilterUpdateStep>)))
  "Returns full string definition for message of type '<FilterUpdateStep>"
  (cl:format cl:nil "# Message detailing a filter update step~%#~%# Fields~%# ======~%# prior_state_cov : State covariance before update~%# prior_obs_error : Observation prediction error before update~%# obs_error_cov   : Theoretical covariance of prediction error before update~%# post_state_cov  : State covariance after update~%# state_delta     : Correction applied to state from update~%# post_obs_error  : Observation prediction error after update~%# obs_jacobian    : Observation function jacobian~%# obs_noise_cov   : Observation noise covariance ~%~%MatrixFloat64 prior_state_cov~%float64[] prior_obs_error~%MatrixFloat64 obs_error_cov~%~%MatrixFloat64 post_state_cov~%float64[] state_delta~%float64[] post_obs_error~%~%MatrixFloat64 obs_jacobian~%MatrixFloat64 obs_noise_cov~%================================================================================~%MSG: argus_msgs/MatrixFloat64~%# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FilterUpdateStep)))
  "Returns full string definition for message of type 'FilterUpdateStep"
  (cl:format cl:nil "# Message detailing a filter update step~%#~%# Fields~%# ======~%# prior_state_cov : State covariance before update~%# prior_obs_error : Observation prediction error before update~%# obs_error_cov   : Theoretical covariance of prediction error before update~%# post_state_cov  : State covariance after update~%# state_delta     : Correction applied to state from update~%# post_obs_error  : Observation prediction error after update~%# obs_jacobian    : Observation function jacobian~%# obs_noise_cov   : Observation noise covariance ~%~%MatrixFloat64 prior_state_cov~%float64[] prior_obs_error~%MatrixFloat64 obs_error_cov~%~%MatrixFloat64 post_state_cov~%float64[] state_delta~%float64[] post_obs_error~%~%MatrixFloat64 obs_jacobian~%MatrixFloat64 obs_noise_cov~%================================================================================~%MSG: argus_msgs/MatrixFloat64~%# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FilterUpdateStep>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'prior_state_cov))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'prior_obs_error) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obs_error_cov))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'post_state_cov))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'state_delta) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'post_obs_error) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obs_jacobian))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obs_noise_cov))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FilterUpdateStep>))
  "Converts a ROS message object to a list"
  (cl:list 'FilterUpdateStep
    (cl:cons ':prior_state_cov (prior_state_cov msg))
    (cl:cons ':prior_obs_error (prior_obs_error msg))
    (cl:cons ':obs_error_cov (obs_error_cov msg))
    (cl:cons ':post_state_cov (post_state_cov msg))
    (cl:cons ':state_delta (state_delta msg))
    (cl:cons ':post_obs_error (post_obs_error msg))
    (cl:cons ':obs_jacobian (obs_jacobian msg))
    (cl:cons ':obs_noise_cov (obs_noise_cov msg))
))
