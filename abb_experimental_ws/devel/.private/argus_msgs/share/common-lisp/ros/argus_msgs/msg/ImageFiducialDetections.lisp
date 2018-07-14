; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude ImageFiducialDetections.msg.html

(cl:defclass <ImageFiducialDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector argus_msgs-msg:FiducialDetection)
   :initform (cl:make-array 0 :element-type 'argus_msgs-msg:FiducialDetection :initial-element (cl:make-instance 'argus_msgs-msg:FiducialDetection))))
)

(cl:defclass ImageFiducialDetections (<ImageFiducialDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageFiducialDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageFiducialDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<ImageFiducialDetections> is deprecated: use argus_msgs-msg:ImageFiducialDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImageFiducialDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:header-val is deprecated.  Use argus_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <ImageFiducialDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:detections-val is deprecated.  Use argus_msgs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageFiducialDetections>) ostream)
  "Serializes a message object of type '<ImageFiducialDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageFiducialDetections>) istream)
  "Deserializes a message object of type '<ImageFiducialDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'argus_msgs-msg:FiducialDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageFiducialDetections>)))
  "Returns string type for a message object of type '<ImageFiducialDetections>"
  "argus_msgs/ImageFiducialDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageFiducialDetections)))
  "Returns string type for a message object of type 'ImageFiducialDetections"
  "argus_msgs/ImageFiducialDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageFiducialDetections>)))
  "Returns md5sum for a message object of type '<ImageFiducialDetections>"
  "6f314965852957bf2ee56e7d38c9d340")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageFiducialDetections)))
  "Returns md5sum for a message object of type 'ImageFiducialDetections"
  "6f314965852957bf2ee56e7d38c9d340")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageFiducialDetections>)))
  "Returns full string definition for message of type '<ImageFiducialDetections>"
  (cl:format cl:nil "# Represents a set of fiducial detections from a single image~%# Timestamp and frame_id are from source image~%std_msgs/Header header~%~%# Detections found in the image~%FiducialDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: argus_msgs/FiducialDetection~%# The name of this fiducial~%string name~%~%# Whether the points are undistorted~%bool undistorted~%~%# Whether the points are in normalized image coordinates~%bool normalized~%~%# The detected points~%Point2D[] points~%~%================================================================================~%MSG: argus_msgs/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageFiducialDetections)))
  "Returns full string definition for message of type 'ImageFiducialDetections"
  (cl:format cl:nil "# Represents a set of fiducial detections from a single image~%# Timestamp and frame_id are from source image~%std_msgs/Header header~%~%# Detections found in the image~%FiducialDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: argus_msgs/FiducialDetection~%# The name of this fiducial~%string name~%~%# Whether the points are undistorted~%bool undistorted~%~%# Whether the points are in normalized image coordinates~%bool normalized~%~%# The detected points~%Point2D[] points~%~%================================================================================~%MSG: argus_msgs/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageFiducialDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageFiducialDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageFiducialDetections
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
