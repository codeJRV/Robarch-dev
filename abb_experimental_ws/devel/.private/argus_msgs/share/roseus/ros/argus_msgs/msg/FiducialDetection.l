;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::FiducialDetection)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'FiducialDetection (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::FIDUCIALDETECTION")
  (make-package "ARGUS_MSGS::FIDUCIALDETECTION"))

(in-package "ROS")
;;//! \htmlinclude FiducialDetection.msg.html


(defclass argus_msgs::FiducialDetection
  :super ros::object
  :slots (_name _undistorted _normalized _points ))

(defmethod argus_msgs::FiducialDetection
  (:init
   (&key
    ((:name __name) "")
    ((:undistorted __undistorted) nil)
    ((:normalized __normalized) nil)
    ((:points __points) (let (r) (dotimes (i 0) (push (instance argus_msgs::Point2D :init) r)) r))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _undistorted __undistorted)
   (setq _normalized __normalized)
   (setq _points __points)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:undistorted
   (&optional __undistorted)
   (if __undistorted (setq _undistorted __undistorted)) _undistorted)
  (:normalized
   (&optional __normalized)
   (if __normalized (setq _normalized __normalized)) _normalized)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; bool _undistorted
    1
    ;; bool _normalized
    1
    ;; argus_msgs/Point2D[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _undistorted
       (if _undistorted (write-byte -1 s) (write-byte 0 s))
     ;; bool _normalized
       (if _normalized (write-byte -1 s) (write-byte 0 s))
     ;; argus_msgs/Point2D[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _undistorted
     (setq _undistorted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _normalized
     (setq _normalized (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; argus_msgs/Point2D[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance argus_msgs::Point2D :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get argus_msgs::FiducialDetection :md5sum-) "9798b72f528ec3822efac6e4bf3400a7")
(setf (get argus_msgs::FiducialDetection :datatype-) "argus_msgs/FiducialDetection")
(setf (get argus_msgs::FiducialDetection :definition-)
      "# The name of this fiducial
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



(provide :argus_msgs/FiducialDetection "9798b72f528ec3822efac6e4bf3400a7")


