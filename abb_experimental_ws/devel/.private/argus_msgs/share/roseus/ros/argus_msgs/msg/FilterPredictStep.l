;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::FilterPredictStep)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'FilterPredictStep (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::FILTERPREDICTSTEP")
  (make-package "ARGUS_MSGS::FILTERPREDICTSTEP"))

(in-package "ROS")
;;//! \htmlinclude FilterPredictStep.msg.html


(defclass argus_msgs::FilterPredictStep
  :super ros::object
  :slots (_step_dt _trans_jacobian _trans_noise_cov _prior_state_cov _post_state_cov ))

(defmethod argus_msgs::FilterPredictStep
  (:init
   (&key
    ((:step_dt __step_dt) 0.0)
    ((:trans_jacobian __trans_jacobian) (instance argus_msgs::MatrixFloat64 :init))
    ((:trans_noise_cov __trans_noise_cov) (instance argus_msgs::MatrixFloat64 :init))
    ((:prior_state_cov __prior_state_cov) (instance argus_msgs::MatrixFloat64 :init))
    ((:post_state_cov __post_state_cov) (instance argus_msgs::MatrixFloat64 :init))
    )
   (send-super :init)
   (setq _step_dt (float __step_dt))
   (setq _trans_jacobian __trans_jacobian)
   (setq _trans_noise_cov __trans_noise_cov)
   (setq _prior_state_cov __prior_state_cov)
   (setq _post_state_cov __post_state_cov)
   self)
  (:step_dt
   (&optional __step_dt)
   (if __step_dt (setq _step_dt __step_dt)) _step_dt)
  (:trans_jacobian
   (&rest __trans_jacobian)
   (if (keywordp (car __trans_jacobian))
       (send* _trans_jacobian __trans_jacobian)
     (progn
       (if __trans_jacobian (setq _trans_jacobian (car __trans_jacobian)))
       _trans_jacobian)))
  (:trans_noise_cov
   (&rest __trans_noise_cov)
   (if (keywordp (car __trans_noise_cov))
       (send* _trans_noise_cov __trans_noise_cov)
     (progn
       (if __trans_noise_cov (setq _trans_noise_cov (car __trans_noise_cov)))
       _trans_noise_cov)))
  (:prior_state_cov
   (&rest __prior_state_cov)
   (if (keywordp (car __prior_state_cov))
       (send* _prior_state_cov __prior_state_cov)
     (progn
       (if __prior_state_cov (setq _prior_state_cov (car __prior_state_cov)))
       _prior_state_cov)))
  (:post_state_cov
   (&rest __post_state_cov)
   (if (keywordp (car __post_state_cov))
       (send* _post_state_cov __post_state_cov)
     (progn
       (if __post_state_cov (setq _post_state_cov (car __post_state_cov)))
       _post_state_cov)))
  (:serialization-length
   ()
   (+
    ;; float64 _step_dt
    8
    ;; argus_msgs/MatrixFloat64 _trans_jacobian
    (send _trans_jacobian :serialization-length)
    ;; argus_msgs/MatrixFloat64 _trans_noise_cov
    (send _trans_noise_cov :serialization-length)
    ;; argus_msgs/MatrixFloat64 _prior_state_cov
    (send _prior_state_cov :serialization-length)
    ;; argus_msgs/MatrixFloat64 _post_state_cov
    (send _post_state_cov :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _step_dt
       (sys::poke _step_dt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; argus_msgs/MatrixFloat64 _trans_jacobian
       (send _trans_jacobian :serialize s)
     ;; argus_msgs/MatrixFloat64 _trans_noise_cov
       (send _trans_noise_cov :serialize s)
     ;; argus_msgs/MatrixFloat64 _prior_state_cov
       (send _prior_state_cov :serialize s)
     ;; argus_msgs/MatrixFloat64 _post_state_cov
       (send _post_state_cov :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _step_dt
     (setq _step_dt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; argus_msgs/MatrixFloat64 _trans_jacobian
     (send _trans_jacobian :deserialize buf ptr-) (incf ptr- (send _trans_jacobian :serialization-length))
   ;; argus_msgs/MatrixFloat64 _trans_noise_cov
     (send _trans_noise_cov :deserialize buf ptr-) (incf ptr- (send _trans_noise_cov :serialization-length))
   ;; argus_msgs/MatrixFloat64 _prior_state_cov
     (send _prior_state_cov :deserialize buf ptr-) (incf ptr- (send _prior_state_cov :serialization-length))
   ;; argus_msgs/MatrixFloat64 _post_state_cov
     (send _post_state_cov :deserialize buf ptr-) (incf ptr- (send _post_state_cov :serialization-length))
   ;;
   self)
  )

(setf (get argus_msgs::FilterPredictStep :md5sum-) "b900620feacd2fcbfc52c9eef367d6d3")
(setf (get argus_msgs::FilterPredictStep :datatype-) "argus_msgs/FilterPredictStep")
(setf (get argus_msgs::FilterPredictStep :definition-)
      "# Message detailing a filter predict step
#
# Fields
# ======
# step_dt         : The predict time step size
# trans_jacobian  : Transition function jacobian
# trans_noise_cov : Transition noise covariance
# prior_state_cov : State covariance before predict
# post_state_cov  : State covariance after predict 

float64 step_dt 
MatrixFloat64 trans_jacobian
MatrixFloat64 trans_noise_cov
MatrixFloat64 prior_state_cov
MatrixFloat64 post_state_cov
================================================================================
MSG: argus_msgs/MatrixFloat64
# Double-precision dynamic-sized matrix message type
bool column_major
uint32 rows
uint32 cols
float64[] data
")



(provide :argus_msgs/FilterPredictStep "b900620feacd2fcbfc52c9eef367d6d3")


