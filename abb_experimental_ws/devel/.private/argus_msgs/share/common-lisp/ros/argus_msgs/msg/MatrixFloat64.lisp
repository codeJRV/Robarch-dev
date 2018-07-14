; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude MatrixFloat64.msg.html

(cl:defclass <MatrixFloat64> (roslisp-msg-protocol:ros-message)
  ((column_major
    :reader column_major
    :initarg :column_major
    :type cl:boolean
    :initform cl:nil)
   (rows
    :reader rows
    :initarg :rows
    :type cl:integer
    :initform 0)
   (cols
    :reader cols
    :initarg :cols
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MatrixFloat64 (<MatrixFloat64>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MatrixFloat64>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MatrixFloat64)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<MatrixFloat64> is deprecated: use argus_msgs-msg:MatrixFloat64 instead.")))

(cl:ensure-generic-function 'column_major-val :lambda-list '(m))
(cl:defmethod column_major-val ((m <MatrixFloat64>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:column_major-val is deprecated.  Use argus_msgs-msg:column_major instead.")
  (column_major m))

(cl:ensure-generic-function 'rows-val :lambda-list '(m))
(cl:defmethod rows-val ((m <MatrixFloat64>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:rows-val is deprecated.  Use argus_msgs-msg:rows instead.")
  (rows m))

(cl:ensure-generic-function 'cols-val :lambda-list '(m))
(cl:defmethod cols-val ((m <MatrixFloat64>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:cols-val is deprecated.  Use argus_msgs-msg:cols instead.")
  (cols m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <MatrixFloat64>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:data-val is deprecated.  Use argus_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MatrixFloat64>) ostream)
  "Serializes a message object of type '<MatrixFloat64>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'column_major) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'rows)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cols)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cols)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cols)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cols)) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MatrixFloat64>) istream)
  "Deserializes a message object of type '<MatrixFloat64>"
    (cl:setf (cl:slot-value msg 'column_major) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'rows)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cols)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cols)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cols)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cols)) (cl:read-byte istream))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MatrixFloat64>)))
  "Returns string type for a message object of type '<MatrixFloat64>"
  "argus_msgs/MatrixFloat64")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MatrixFloat64)))
  "Returns string type for a message object of type 'MatrixFloat64"
  "argus_msgs/MatrixFloat64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MatrixFloat64>)))
  "Returns md5sum for a message object of type '<MatrixFloat64>"
  "d49ad88054ad35f5431113eea16faa1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MatrixFloat64)))
  "Returns md5sum for a message object of type 'MatrixFloat64"
  "d49ad88054ad35f5431113eea16faa1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MatrixFloat64>)))
  "Returns full string definition for message of type '<MatrixFloat64>"
  (cl:format cl:nil "# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MatrixFloat64)))
  "Returns full string definition for message of type 'MatrixFloat64"
  (cl:format cl:nil "# Double-precision dynamic-sized matrix message type~%bool column_major~%uint32 rows~%uint32 cols~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MatrixFloat64>))
  (cl:+ 0
     1
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MatrixFloat64>))
  "Converts a ROS message object to a list"
  (cl:list 'MatrixFloat64
    (cl:cons ':column_major (column_major msg))
    (cl:cons ':rows (rows msg))
    (cl:cons ':cols (cols msg))
    (cl:cons ':data (data msg))
))
