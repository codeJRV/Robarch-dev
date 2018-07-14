;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::EstimatePerformance)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'EstimatePerformance (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::ESTIMATEPERFORMANCE")
  (make-package "ARGUS_MSGS::ESTIMATEPERFORMANCE"))

(in-package "ROS")
;;//! \htmlinclude EstimatePerformance.msg.html


(defclass argus_msgs::EstimatePerformance
  :super ros::object
  :slots (_rms_linear_vel _rms_angular_vel ))

(defmethod argus_msgs::EstimatePerformance
  (:init
   (&key
    ((:rms_linear_vel __rms_linear_vel) 0.0)
    ((:rms_angular_vel __rms_angular_vel) 0.0)
    )
   (send-super :init)
   (setq _rms_linear_vel (float __rms_linear_vel))
   (setq _rms_angular_vel (float __rms_angular_vel))
   self)
  (:rms_linear_vel
   (&optional __rms_linear_vel)
   (if __rms_linear_vel (setq _rms_linear_vel __rms_linear_vel)) _rms_linear_vel)
  (:rms_angular_vel
   (&optional __rms_angular_vel)
   (if __rms_angular_vel (setq _rms_angular_vel __rms_angular_vel)) _rms_angular_vel)
  (:serialization-length
   ()
   (+
    ;; float64 _rms_linear_vel
    8
    ;; float64 _rms_angular_vel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _rms_linear_vel
       (sys::poke _rms_linear_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rms_angular_vel
       (sys::poke _rms_angular_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _rms_linear_vel
     (setq _rms_linear_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rms_angular_vel
     (setq _rms_angular_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get argus_msgs::EstimatePerformance :md5sum-) "17e5293affcaac44e00d9f001150b5b3")
(setf (get argus_msgs::EstimatePerformance :datatype-) "argus_msgs/EstimatePerformance")
(setf (get argus_msgs::EstimatePerformance :definition-)
      "# Message with state estimate performance

# Root mean square error for linear velocity
float64 rms_linear_vel

# Root mean square error for angular velocity
float64 rms_angular_vel
")



(provide :argus_msgs/EstimatePerformance "17e5293affcaac44e00d9f001150b5b3")


