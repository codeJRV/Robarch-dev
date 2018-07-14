;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::FilterStepInfo)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'FilterStepInfo (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::FILTERSTEPINFO")
  (make-package "ARGUS_MSGS::FILTERSTEPINFO"))

(in-package "ROS")
;;//! \htmlinclude FilterStepInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*PREDICT_STEP*" (find-package "ARGUS_MSGS::FILTERSTEPINFO"))
(shadow '*PREDICT_STEP* (find-package "ARGUS_MSGS::FILTERSTEPINFO"))
(defconstant argus_msgs::FilterStepInfo::*PREDICT_STEP* 0)
(intern "*UPDATE_STEP*" (find-package "ARGUS_MSGS::FILTERSTEPINFO"))
(shadow '*UPDATE_STEP* (find-package "ARGUS_MSGS::FILTERSTEPINFO"))
(defconstant argus_msgs::FilterStepInfo::*UPDATE_STEP* 1)
(defclass argus_msgs::FilterStepInfo
  :super ros::object
  :slots (_header _step_num _info_type _predict _update ))

(defmethod argus_msgs::FilterStepInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:step_num __step_num) 0)
    ((:info_type __info_type) 0)
    ((:predict __predict) (instance argus_msgs::FilterPredictStep :init))
    ((:update __update) (instance argus_msgs::FilterUpdateStep :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _step_num (round __step_num))
   (setq _info_type (round __info_type))
   (setq _predict __predict)
   (setq _update __update)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:step_num
   (&optional __step_num)
   (if __step_num (setq _step_num __step_num)) _step_num)
  (:info_type
   (&optional __info_type)
   (if __info_type (setq _info_type __info_type)) _info_type)
  (:predict
   (&rest __predict)
   (if (keywordp (car __predict))
       (send* _predict __predict)
     (progn
       (if __predict (setq _predict (car __predict)))
       _predict)))
  (:update
   (&rest __update)
   (if (keywordp (car __update))
       (send* _update __update)
     (progn
       (if __update (setq _update (car __update)))
       _update)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _step_num
    8
    ;; uint8 _info_type
    1
    ;; argus_msgs/FilterPredictStep _predict
    (send _predict :serialization-length)
    ;; argus_msgs/FilterUpdateStep _update
    (send _update :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _step_num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _step_num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _step_num) (= (length (_step_num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_step_num . bv) 0) 0) s)
              (write-long (ash (elt (_step_num . bv) 1) -1) s))
             ((and (class _step_num) (= (length (_step_num . bv)) 1)) ;; big1
              (write-long (elt (_step_num . bv) 0) s)
              (write-long (if (>= _step_num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _step_num s)(write-long (if (>= _step_num 0) 0 #xffffffff) s)))
     ;; uint8 _info_type
       (write-byte _info_type s)
     ;; argus_msgs/FilterPredictStep _predict
       (send _predict :serialize s)
     ;; argus_msgs/FilterUpdateStep _update
       (send _update :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _step_num
     
#+(or :alpha :irix6 :x86_64)
      (setf _step_num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _step_num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint8 _info_type
     (setq _info_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; argus_msgs/FilterPredictStep _predict
     (send _predict :deserialize buf ptr-) (incf ptr- (send _predict :serialization-length))
   ;; argus_msgs/FilterUpdateStep _update
     (send _update :deserialize buf ptr-) (incf ptr- (send _update :serialization-length))
   ;;
   self)
  )

(setf (get argus_msgs::FilterStepInfo :md5sum-) "7f102f1a0b841c1cdbe4dbe82e22681e")
(setf (get argus_msgs::FilterStepInfo :datatype-) "argus_msgs/FilterStepInfo")
(setf (get argus_msgs::FilterStepInfo :definition-)
      "# Message detailing a filter predict or update step
#
# Fields
# ======
# header/stamp     : Time at which this step ends or occurs at
# header/frame_id  : This observation source's name [unused for predict]
# step_num  : Filter step index
# info_type : Enumerated active payload type
# predict   : Predict info payload
# update    : Update info payload

# Defines the active payload field
uint8 PREDICT_STEP = 0
uint8 UPDATE_STEP = 1

std_msgs/Header header
uint64 step_num
uint8 info_type
argus_msgs/FilterPredictStep predict
argus_msgs/FilterUpdateStep update

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
MSG: argus_msgs/FilterPredictStep
# Message detailing a filter predict step
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
================================================================================
MSG: argus_msgs/FilterUpdateStep
# Message detailing a filter update step
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
")



(provide :argus_msgs/FilterStepInfo "7f102f1a0b841c1cdbe4dbe82e22681e")


