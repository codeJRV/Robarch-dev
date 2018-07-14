; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude SymmetricFloat64.msg.html

(cl:defclass <SymmetricFloat64> (roslisp-msg-protocol:ros-message)
  ((dim
    :reader dim
    :initarg :dim
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SymmetricFloat64 (<SymmetricFloat64>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SymmetricFloat64>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SymmetricFloat64)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<SymmetricFloat64> is deprecated: use argus_msgs-msg:SymmetricFloat64 instead.")))

(cl:ensure-generic-function 'dim-val :lambda-list '(m))
(cl:defmethod dim-val ((m <SymmetricFloat64>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:dim-val is deprecated.  Use argus_msgs-msg:dim instead.")
  (dim m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SymmetricFloat64>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:data-val is deprecated.  Use argus_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SymmetricFloat64>) ostream)
  "Serializes a message object of type '<SymmetricFloat64>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dim)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dim)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dim)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dim)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
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
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SymmetricFloat64>) istream)
  "Deserializes a message object of type '<SymmetricFloat64>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dim)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dim)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dim)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dim)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SymmetricFloat64>)))
  "Returns string type for a message object of type '<SymmetricFloat64>"
  "argus_msgs/SymmetricFloat64")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymmetricFloat64)))
  "Returns string type for a message object of type 'SymmetricFloat64"
  "argus_msgs/SymmetricFloat64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SymmetricFloat64>)))
  "Returns md5sum for a message object of type '<SymmetricFloat64>"
  "099dfa5da2b4d88d46f9e50c2fc822ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SymmetricFloat64)))
  "Returns md5sum for a message object of type 'SymmetricFloat64"
  "099dfa5da2b4d88d46f9e50c2fc822ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SymmetricFloat64>)))
  "Returns full string definition for message of type '<SymmetricFloat64>"
  (cl:format cl:nil "# Double-precision symmetric matrix message type~%uint32 dim # Rows = cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SymmetricFloat64)))
  "Returns full string definition for message of type 'SymmetricFloat64"
  (cl:format cl:nil "# Double-precision symmetric matrix message type~%uint32 dim # Rows = cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SymmetricFloat64>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SymmetricFloat64>))
  "Converts a ROS message object to a list"
  (cl:list 'SymmetricFloat64
    (cl:cons ':dim (dim msg))
    (cl:cons ':data (data msg))
))
