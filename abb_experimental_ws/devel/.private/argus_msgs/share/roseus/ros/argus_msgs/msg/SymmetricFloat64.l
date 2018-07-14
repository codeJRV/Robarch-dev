;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::SymmetricFloat64)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'SymmetricFloat64 (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::SYMMETRICFLOAT64")
  (make-package "ARGUS_MSGS::SYMMETRICFLOAT64"))

(in-package "ROS")
;;//! \htmlinclude SymmetricFloat64.msg.html


(defclass argus_msgs::SymmetricFloat64
  :super ros::object
  :slots (_dim _data ))

(defmethod argus_msgs::SymmetricFloat64
  (:init
   (&key
    ((:dim __dim) 0)
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _dim (round __dim))
   (setq _data __data)
   self)
  (:dim
   (&optional __dim)
   (if __dim (setq _dim __dim)) _dim)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; uint32 _dim
    4
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _dim
       (write-long _dim s)
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _dim
     (setq _dim (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get argus_msgs::SymmetricFloat64 :md5sum-) "099dfa5da2b4d88d46f9e50c2fc822ad")
(setf (get argus_msgs::SymmetricFloat64 :datatype-) "argus_msgs/SymmetricFloat64")
(setf (get argus_msgs::SymmetricFloat64 :definition-)
      "# Double-precision symmetric matrix message type
uint32 dim # Rows = cols
float64[] data
")



(provide :argus_msgs/SymmetricFloat64 "099dfa5da2b4d88d46f9e50c2fc822ad")


