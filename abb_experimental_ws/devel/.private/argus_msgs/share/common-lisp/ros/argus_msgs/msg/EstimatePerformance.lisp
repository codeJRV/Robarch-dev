; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude EstimatePerformance.msg.html

(cl:defclass <EstimatePerformance> (roslisp-msg-protocol:ros-message)
  ((rms_linear_vel
    :reader rms_linear_vel
    :initarg :rms_linear_vel
    :type cl:float
    :initform 0.0)
   (rms_angular_vel
    :reader rms_angular_vel
    :initarg :rms_angular_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass EstimatePerformance (<EstimatePerformance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EstimatePerformance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EstimatePerformance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<EstimatePerformance> is deprecated: use argus_msgs-msg:EstimatePerformance instead.")))

(cl:ensure-generic-function 'rms_linear_vel-val :lambda-list '(m))
(cl:defmethod rms_linear_vel-val ((m <EstimatePerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:rms_linear_vel-val is deprecated.  Use argus_msgs-msg:rms_linear_vel instead.")
  (rms_linear_vel m))

(cl:ensure-generic-function 'rms_angular_vel-val :lambda-list '(m))
(cl:defmethod rms_angular_vel-val ((m <EstimatePerformance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:rms_angular_vel-val is deprecated.  Use argus_msgs-msg:rms_angular_vel instead.")
  (rms_angular_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EstimatePerformance>) ostream)
  "Serializes a message object of type '<EstimatePerformance>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rms_linear_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rms_angular_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EstimatePerformance>) istream)
  "Deserializes a message object of type '<EstimatePerformance>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rms_linear_vel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rms_angular_vel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EstimatePerformance>)))
  "Returns string type for a message object of type '<EstimatePerformance>"
  "argus_msgs/EstimatePerformance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EstimatePerformance)))
  "Returns string type for a message object of type 'EstimatePerformance"
  "argus_msgs/EstimatePerformance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EstimatePerformance>)))
  "Returns md5sum for a message object of type '<EstimatePerformance>"
  "17e5293affcaac44e00d9f001150b5b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EstimatePerformance)))
  "Returns md5sum for a message object of type 'EstimatePerformance"
  "17e5293affcaac44e00d9f001150b5b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EstimatePerformance>)))
  "Returns full string definition for message of type '<EstimatePerformance>"
  (cl:format cl:nil "# Message with state estimate performance~%~%# Root mean square error for linear velocity~%float64 rms_linear_vel~%~%# Root mean square error for angular velocity~%float64 rms_angular_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EstimatePerformance)))
  "Returns full string definition for message of type 'EstimatePerformance"
  (cl:format cl:nil "# Message with state estimate performance~%~%# Root mean square error for linear velocity~%float64 rms_linear_vel~%~%# Root mean square error for angular velocity~%float64 rms_angular_vel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EstimatePerformance>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EstimatePerformance>))
  "Converts a ROS message object to a list"
  (cl:list 'EstimatePerformance
    (cl:cons ':rms_linear_vel (rms_linear_vel msg))
    (cl:cons ':rms_angular_vel (rms_angular_vel msg))
))
