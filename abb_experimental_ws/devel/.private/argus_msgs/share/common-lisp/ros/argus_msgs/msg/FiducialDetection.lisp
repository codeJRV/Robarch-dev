; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude FiducialDetection.msg.html

(cl:defclass <FiducialDetection> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (undistorted
    :reader undistorted
    :initarg :undistorted
    :type cl:boolean
    :initform cl:nil)
   (normalized
    :reader normalized
    :initarg :normalized
    :type cl:boolean
    :initform cl:nil)
   (points
    :reader points
    :initarg :points
    :type (cl:vector argus_msgs-msg:Point2D)
   :initform (cl:make-array 0 :element-type 'argus_msgs-msg:Point2D :initial-element (cl:make-instance 'argus_msgs-msg:Point2D))))
)

(cl:defclass FiducialDetection (<FiducialDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FiducialDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FiducialDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<FiducialDetection> is deprecated: use argus_msgs-msg:FiducialDetection instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <FiducialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:name-val is deprecated.  Use argus_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'undistorted-val :lambda-list '(m))
(cl:defmethod undistorted-val ((m <FiducialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:undistorted-val is deprecated.  Use argus_msgs-msg:undistorted instead.")
  (undistorted m))

(cl:ensure-generic-function 'normalized-val :lambda-list '(m))
(cl:defmethod normalized-val ((m <FiducialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:normalized-val is deprecated.  Use argus_msgs-msg:normalized instead.")
  (normalized m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <FiducialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:points-val is deprecated.  Use argus_msgs-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FiducialDetection>) ostream)
  "Serializes a message object of type '<FiducialDetection>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'undistorted) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'normalized) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FiducialDetection>) istream)
  "Deserializes a message object of type '<FiducialDetection>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'undistorted) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'normalized) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'argus_msgs-msg:Point2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FiducialDetection>)))
  "Returns string type for a message object of type '<FiducialDetection>"
  "argus_msgs/FiducialDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FiducialDetection)))
  "Returns string type for a message object of type 'FiducialDetection"
  "argus_msgs/FiducialDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FiducialDetection>)))
  "Returns md5sum for a message object of type '<FiducialDetection>"
  "9798b72f528ec3822efac6e4bf3400a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FiducialDetection)))
  "Returns md5sum for a message object of type 'FiducialDetection"
  "9798b72f528ec3822efac6e4bf3400a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FiducialDetection>)))
  "Returns full string definition for message of type '<FiducialDetection>"
  (cl:format cl:nil "# The name of this fiducial~%string name~%~%# Whether the points are undistorted~%bool undistorted~%~%# Whether the points are in normalized image coordinates~%bool normalized~%~%# The detected points~%Point2D[] points~%~%================================================================================~%MSG: argus_msgs/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FiducialDetection)))
  "Returns full string definition for message of type 'FiducialDetection"
  (cl:format cl:nil "# The name of this fiducial~%string name~%~%# Whether the points are undistorted~%bool undistorted~%~%# Whether the points are in normalized image coordinates~%bool normalized~%~%# The detected points~%Point2D[] points~%~%================================================================================~%MSG: argus_msgs/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FiducialDetection>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FiducialDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'FiducialDetection
    (cl:cons ':name (name msg))
    (cl:cons ':undistorted (undistorted msg))
    (cl:cons ':normalized (normalized msg))
    (cl:cons ':points (points msg))
))
