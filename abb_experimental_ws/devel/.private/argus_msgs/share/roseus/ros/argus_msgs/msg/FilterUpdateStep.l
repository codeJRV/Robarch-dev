;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::FilterUpdateStep)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'FilterUpdateStep (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::FILTERUPDATESTEP")
  (make-package "ARGUS_MSGS::FILTERUPDATESTEP"))

(in-package "ROS")
;;//! \htmlinclude FilterUpdateStep.msg.html


(defclass argus_msgs::FilterUpdateStep
  :super ros::object
  :slots (_prior_state_cov _prior_obs_error _obs_error_cov _post_state_cov _state_delta _post_obs_error _obs_jacobian _obs_noise_cov ))

(defmethod argus_msgs::FilterUpdateStep
  (:init
   (&key
    ((:prior_state_cov __prior_state_cov) (instance argus_msgs::MatrixFloat64 :init))
    ((:prior_obs_error __prior_obs_error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:obs_error_cov __obs_error_cov) (instance argus_msgs::MatrixFloat64 :init))
    ((:post_state_cov __post_state_cov) (instance argus_msgs::MatrixFloat64 :init))
    ((:state_delta __state_delta) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:post_obs_error __post_obs_error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:obs_jacobian __obs_jacobian) (instance argus_msgs::MatrixFloat64 :init))
    ((:obs_noise_cov __obs_noise_cov) (instance argus_msgs::MatrixFloat64 :init))
    )
   (send-super :init)
   (setq _prior_state_cov __prior_state_cov)
   (setq _prior_obs_error __prior_obs_error)
   (setq _obs_error_cov __obs_error_cov)
   (setq _post_state_cov __post_state_cov)
   (setq _state_delta __state_delta)
   (setq _post_obs_error __post_obs_error)
   (setq _obs_jacobian __obs_jacobian)
   (setq _obs_noise_cov __obs_noise_cov)
   self)
  (:prior_state_cov
   (&rest __prior_state_cov)
   (if (keywordp (car __prior_state_cov))
       (send* _prior_state_cov __prior_state_cov)
     (progn
       (if __prior_state_cov (setq _prior_state_cov (car __prior_state_cov)))
       _prior_state_cov)))
  (:prior_obs_error
   (&optional __prior_obs_error)
   (if __prior_obs_error (setq _prior_obs_error __prior_obs_error)) _prior_obs_error)
  (:obs_error_cov
   (&rest __obs_error_cov)
   (if (keywordp (car __obs_error_cov))
       (send* _obs_error_cov __obs_error_cov)
     (progn
       (if __obs_error_cov (setq _obs_error_cov (car __obs_error_cov)))
       _obs_error_cov)))
  (:post_state_cov
   (&rest __post_state_cov)
   (if (keywordp (car __post_state_cov))
       (send* _post_state_cov __post_state_cov)
     (progn
       (if __post_state_cov (setq _post_state_cov (car __post_state_cov)))
       _post_state_cov)))
  (:state_delta
   (&optional __state_delta)
   (if __state_delta (setq _state_delta __state_delta)) _state_delta)
  (:post_obs_error
   (&optional __post_obs_error)
   (if __post_obs_error (setq _post_obs_error __post_obs_error)) _post_obs_error)
  (:obs_jacobian
   (&rest __obs_jacobian)
   (if (keywordp (car __obs_jacobian))
       (send* _obs_jacobian __obs_jacobian)
     (progn
       (if __obs_jacobian (setq _obs_jacobian (car __obs_jacobian)))
       _obs_jacobian)))
  (:obs_noise_cov
   (&rest __obs_noise_cov)
   (if (keywordp (car __obs_noise_cov))
       (send* _obs_noise_cov __obs_noise_cov)
     (progn
       (if __obs_noise_cov (setq _obs_noise_cov (car __obs_noise_cov)))
       _obs_noise_cov)))
  (:serialization-length
   ()
   (+
    ;; argus_msgs/MatrixFloat64 _prior_state_cov
    (send _prior_state_cov :serialization-length)
    ;; float64[] _prior_obs_error
    (* 8    (length _prior_obs_error)) 4
    ;; argus_msgs/MatrixFloat64 _obs_error_cov
    (send _obs_error_cov :serialization-length)
    ;; argus_msgs/MatrixFloat64 _post_state_cov
    (send _post_state_cov :serialization-length)
    ;; float64[] _state_delta
    (* 8    (length _state_delta)) 4
    ;; float64[] _post_obs_error
    (* 8    (length _post_obs_error)) 4
    ;; argus_msgs/MatrixFloat64 _obs_jacobian
    (send _obs_jacobian :serialization-length)
    ;; argus_msgs/MatrixFloat64 _obs_noise_cov
    (send _obs_noise_cov :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; argus_msgs/MatrixFloat64 _prior_state_cov
       (send _prior_state_cov :serialize s)
     ;; float64[] _prior_obs_error
     (write-long (length _prior_obs_error) s)
     (dotimes (i (length _prior_obs_error))
       (sys::poke (elt _prior_obs_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; argus_msgs/MatrixFloat64 _obs_error_cov
       (send _obs_error_cov :serialize s)
     ;; argus_msgs/MatrixFloat64 _post_state_cov
       (send _post_state_cov :serialize s)
     ;; float64[] _state_delta
     (write-long (length _state_delta) s)
     (dotimes (i (length _state_delta))
       (sys::poke (elt _state_delta i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _post_obs_error
     (write-long (length _post_obs_error) s)
     (dotimes (i (length _post_obs_error))
       (sys::poke (elt _post_obs_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; argus_msgs/MatrixFloat64 _obs_jacobian
       (send _obs_jacobian :serialize s)
     ;; argus_msgs/MatrixFloat64 _obs_noise_cov
       (send _obs_noise_cov :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; argus_msgs/MatrixFloat64 _prior_state_cov
     (send _prior_state_cov :deserialize buf ptr-) (incf ptr- (send _prior_state_cov :serialization-length))
   ;; float64[] _prior_obs_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _prior_obs_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _prior_obs_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; argus_msgs/MatrixFloat64 _obs_error_cov
     (send _obs_error_cov :deserialize buf ptr-) (incf ptr- (send _obs_error_cov :serialization-length))
   ;; argus_msgs/MatrixFloat64 _post_state_cov
     (send _post_state_cov :deserialize buf ptr-) (incf ptr- (send _post_state_cov :serialization-length))
   ;; float64[] _state_delta
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state_delta (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _state_delta i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _post_obs_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _post_obs_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _post_obs_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; argus_msgs/MatrixFloat64 _obs_jacobian
     (send _obs_jacobian :deserialize buf ptr-) (incf ptr- (send _obs_jacobian :serialization-length))
   ;; argus_msgs/MatrixFloat64 _obs_noise_cov
     (send _obs_noise_cov :deserialize buf ptr-) (incf ptr- (send _obs_noise_cov :serialization-length))
   ;;
   self)
  )

(setf (get argus_msgs::FilterUpdateStep :md5sum-) "451710d77b207f2e2068272b8ce64023")
(setf (get argus_msgs::FilterUpdateStep :datatype-) "argus_msgs/FilterUpdateStep")
(setf (get argus_msgs::FilterUpdateStep :definition-)
      "# Message detailing a filter update step
#
# Fields
# ======
# prior_state_cov : State covariance before update
# prior_obs_error : Observation prediction error before update
# obs_error_cov   : Theoretical covariance of prediction error before update
# post_state_cov  : State covariance after update
# state_delta     : Correction applied to state from update
# post_obs_error  : Observation prediction error after update
# obs_jacobian    : Observation function jacobian
# obs_noise_cov   : Observation noise covariance 

MatrixFloat64 prior_state_cov
float64[] prior_obs_error
MatrixFloat64 obs_error_cov

MatrixFloat64 post_state_cov
float64[] state_delta
float64[] post_obs_error

MatrixFloat64 obs_jacobian
MatrixFloat64 obs_noise_cov
================================================================================
MSG: argus_msgs/MatrixFloat64
# Double-precision dynamic-sized matrix message type
bool column_major
uint32 rows
uint32 cols
float64[] data
")



(provide :argus_msgs/FilterUpdateStep "451710d77b207f2e2068272b8ce64023")


