;; Auto-generated. Do not edit!


(when (boundp 'argus_msgs::MatrixFloat64)
  (if (not (find-package "ARGUS_MSGS"))
    (make-package "ARGUS_MSGS"))
  (shadow 'MatrixFloat64 (find-package "ARGUS_MSGS")))
(unless (find-package "ARGUS_MSGS::MATRIXFLOAT64")
  (make-package "ARGUS_MSGS::MATRIXFLOAT64"))

(in-package "ROS")
;;//! \htmlinclude MatrixFloat64.msg.html


(defclass argus_msgs::MatrixFloat64
  :super ros::object
  :slots (_column_major _rows _cols _data ))

(defmethod argus_msgs::MatrixFloat64
  (:init
   (&key
    ((:column_major __column_major) nil)
    ((:rows __rows) 0)
    ((:cols __cols) 0)
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _column_major __column_major)
   (setq _rows (round __rows))
   (setq _cols (round __cols))
   (setq _data __data)
   self)
  (:column_major
   (&optional __column_major)
   (if __column_major (setq _column_major __column_major)) _column_major)
  (:rows
   (&optional __rows)
   (if __rows (setq _rows __rows)) _rows)
  (:cols
   (&optional __cols)
   (if __cols (setq _cols __cols)) _cols)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _column_major
    1
    ;; uint32 _rows
    4
    ;; uint32 _cols
    4
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _column_major
       (if _column_major (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _rows
       (write-long _rows s)
     ;; uint32 _cols
       (write-long _cols s)
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _column_major
     (setq _column_major (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _rows
     (setq _rows (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _cols
     (setq _cols (sys::peek buf ptr- :integer)) (incf ptr- 4)
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

(setf (get argus_msgs::MatrixFloat64 :md5sum-) "d49ad88054ad35f5431113eea16faa1f")
(setf (get argus_msgs::MatrixFloat64 :datatype-) "argus_msgs/MatrixFloat64")
(setf (get argus_msgs::MatrixFloat64 :definition-)
      "# Double-precision dynamic-sized matrix message type
bool column_major
uint32 rows
uint32 cols
float64[] data
")



(provide :argus_msgs/MatrixFloat64 "d49ad88054ad35f5431113eea16faa1f")


