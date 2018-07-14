; Auto-generated. Do not edit!


(cl:in-package argus_msgs-msg)


;//! \htmlinclude TransformWithCovarianceStamped.msg.html

(cl:defclass <TransformWithCovarianceStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (transform
    :reader transform
    :initarg :transform
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 36 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass TransformWithCovarianceStamped (<TransformWithCovarianceStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformWithCovarianceStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformWithCovarianceStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argus_msgs-msg:<TransformWithCovarianceStamped> is deprecated: use argus_msgs-msg:TransformWithCovarianceStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TransformWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:header-val is deprecated.  Use argus_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'transform-val :lambda-list '(m))
(cl:defmethod transform-val ((m <TransformWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:transform-val is deprecated.  Use argus_msgs-msg:transform instead.")
  (transform m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <TransformWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argus_msgs-msg:covariance-val is deprecated.  Use argus_msgs-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformWithCovarianceStamped>) ostream)
  "Serializes a message object of type '<TransformWithCovarianceStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transform) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformWithCovarianceStamped>) istream)
  "Deserializes a message object of type '<TransformWithCovarianceStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transform) istream)
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 36))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 36)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformWithCovarianceStamped>)))
  "Returns string type for a message object of type '<TransformWithCovarianceStamped>"
  "argus_msgs/TransformWithCovarianceStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformWithCovarianceStamped)))
  "Returns string type for a message object of type 'TransformWithCovarianceStamped"
  "argus_msgs/TransformWithCovarianceStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformWithCovarianceStamped>)))
  "Returns md5sum for a message object of type '<TransformWithCovarianceStamped>"
  "eb5865d296e092b071b42b179ac140e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformWithCovarianceStamped)))
  "Returns md5sum for a message object of type 'TransformWithCovarianceStamped"
  "eb5865d296e092b071b42b179ac140e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformWithCovarianceStamped>)))
  "Returns full string definition for message of type '<TransformWithCovarianceStamped>"
  (cl:format cl:nil "# Message header~%std_msgs/Header header~%~%# The transform~%geometry_msgs/Transform transform~%~%# Covariance associated with transform~%float64[36] covariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformWithCovarianceStamped)))
  "Returns full string definition for message of type 'TransformWithCovarianceStamped"
  (cl:format cl:nil "# Message header~%std_msgs/Header header~%~%# The transform~%geometry_msgs/Transform transform~%~%# Covariance associated with transform~%float64[36] covariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformWithCovarianceStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transform))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformWithCovarianceStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformWithCovarianceStamped
    (cl:cons ':header (header msg))
    (cl:cons ':transform (transform msg))
    (cl:cons ':covariance (covariance msg))
))
