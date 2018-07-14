# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "argus_msgs: 11 messages, 0 services")

set(MSG_I_FLAGS "-Iargus_msgs:/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(argus_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg" "argus_msgs/MatrixFloat64"
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg" ""
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg" "argus_msgs/FilterUpdateStep:argus_msgs/FilterPredictStep:std_msgs/Header:argus_msgs/MatrixFloat64"
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg" ""
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg" "argus_msgs/Point2D"
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg" "geometry_msgs/Quaternion:geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Transform"
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg" ""
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg" "argus_msgs/MatrixFloat64"
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg" ""
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg" "geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/TwistWithCovariance:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:nav_msgs/Odometry"
)

get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg" NAME_WE)
add_custom_target(_argus_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "argus_msgs" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg" "argus_msgs/FiducialDetection:argus_msgs/Point2D:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)
_generate_msg_cpp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(argus_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(argus_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(argus_msgs_generate_messages argus_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_cpp _argus_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(argus_msgs_gencpp)
add_dependencies(argus_msgs_gencpp argus_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS argus_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)
_generate_msg_eus(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(argus_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(argus_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(argus_msgs_generate_messages argus_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_eus _argus_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(argus_msgs_geneus)
add_dependencies(argus_msgs_geneus argus_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS argus_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)
_generate_msg_lisp(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(argus_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(argus_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(argus_msgs_generate_messages argus_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_lisp _argus_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(argus_msgs_genlisp)
add_dependencies(argus_msgs_genlisp argus_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS argus_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)
_generate_msg_nodejs(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(argus_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(argus_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(argus_msgs_generate_messages argus_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_nodejs _argus_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(argus_msgs_gennodejs)
add_dependencies(argus_msgs_gennodejs argus_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS argus_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)
_generate_msg_py(argus_msgs
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg"
  "${MSG_I_FLAGS}"
  "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg;/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(argus_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(argus_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(argus_msgs_generate_messages argus_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterPredictStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/EstimatePerformance.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterStepInfo.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/SymmetricFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FiducialDetection.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/TransformWithCovarianceStamped.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/Point2D.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/FilterUpdateStep.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/MatrixFloat64.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/OdometryArray.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/argus_utils/argus_msgs/msg/ImageFiducialDetections.msg" NAME_WE)
add_dependencies(argus_msgs_generate_messages_py _argus_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(argus_msgs_genpy)
add_dependencies(argus_msgs_genpy argus_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS argus_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/argus_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(argus_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(argus_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(argus_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/argus_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(argus_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(argus_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(argus_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/argus_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(argus_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(argus_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(argus_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/argus_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(argus_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(argus_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(argus_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/argus_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(argus_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(argus_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(argus_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
