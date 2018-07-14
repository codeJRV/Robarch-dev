;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::ImageFiducialDetections)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'ImageFiducialDetections (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::IMAGEFIDUCIALDETECTIONS")
  (make-package "ARGUS_MSGS::IMAGEFIDUCIALDETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude ImageFiducialDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass argus_msgs::ImageFiducialDetections
  :super ros::object
  :slots (_header _detections ))

(defmethod argus_msgs::ImageFiducialDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) (let (r) (dotimes (i 0) (push (instance argus_msgs::FiducialDetection :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; argus_msgs/FiducialDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; argus_msgs/FiducialDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; argus_msgs/FiducialDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance argus_msgs::FiducialDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get argus_msgs::ImageFiducialDetections :md5sum-) "6f314965852957bf2ee56e7d38c9d340")
(setf (get argus_msgs::ImageFiducialDetections :datatype-) "argus_msgs/ImageFiducialDetections")
(setf (get argus_msgs::ImageFiducialDetections :definition-)
      "# Represents a set of fiducial detections from a single image
# Timestamp and frame_id are from source image
std_msgs/Header header

# Detections found in the image
FiducialDetection[] detections

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
MSG: argus_msgs/FiducialDetection
# The name of this fiducial
string name

# Whether the points are undistorted
bool undistorted

# Whether the points are in normalized image coordinates
bool normalized

# The detected points
Point2D[] points

================================================================================
MSG: argus_msgs/Point2D
float64 x
float64 y

")



(provide :argus_msgs/ImageFiducialDetections "6f314965852957bf2ee56e7d38c9d340")


