
(cl:in-package :asdf)

(defsystem "argus_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EstimatePerformance" :depends-on ("_package_EstimatePerformance"))
    (:file "_package_EstimatePerformance" :depends-on ("_package"))
    (:file "FiducialDetection" :depends-on ("_package_FiducialDetection"))
    (:file "_package_FiducialDetection" :depends-on ("_package"))
    (:file "FilterPredictStep" :depends-on ("_package_FilterPredictStep"))
    (:file "_package_FilterPredictStep" :depends-on ("_package"))
    (:file "FilterStepInfo" :depends-on ("_package_FilterStepInfo"))
    (:file "_package_FilterStepInfo" :depends-on ("_package"))
    (:file "FilterUpdateStep" :depends-on ("_package_FilterUpdateStep"))
    (:file "_package_FilterUpdateStep" :depends-on ("_package"))
    (:file "ImageFiducialDetections" :depends-on ("_package_ImageFiducialDetections"))
    (:file "_package_ImageFiducialDetections" :depends-on ("_package"))
    (:file "MatrixFloat64" :depends-on ("_package_MatrixFloat64"))
    (:file "_package_MatrixFloat64" :depends-on ("_package"))
    (:file "OdometryArray" :depends-on ("_package_OdometryArray"))
    (:file "_package_OdometryArray" :depends-on ("_package"))
    (:file "Point2D" :depends-on ("_package_Point2D"))
    (:file "_package_Point2D" :depends-on ("_package"))
    (:file "SymmetricFloat64" :depends-on ("_package_SymmetricFloat64"))
    (:file "_package_SymmetricFloat64" :depends-on ("_package"))
    (:file "TransformWithCovarianceStamped" :depends-on ("_package_TransformWithCovarianceStamped"))
    (:file "_package_TransformWithCovarianceStamped" :depends-on ("_package"))
  ))