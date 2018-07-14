# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from argus_msgs/FilterPredictStep.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import argus_msgs.msg

class FilterPredictStep(genpy.Message):
  _md5sum = "b900620feacd2fcbfc52c9eef367d6d3"
  _type = "argus_msgs/FilterPredictStep"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Message detailing a filter predict step
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
float64[] data"""
  __slots__ = ['step_dt','trans_jacobian','trans_noise_cov','prior_state_cov','post_state_cov']
  _slot_types = ['float64','argus_msgs/MatrixFloat64','argus_msgs/MatrixFloat64','argus_msgs/MatrixFloat64','argus_msgs/MatrixFloat64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       step_dt,trans_jacobian,trans_noise_cov,prior_state_cov,post_state_cov

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FilterPredictStep, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.step_dt is None:
        self.step_dt = 0.
      if self.trans_jacobian is None:
        self.trans_jacobian = argus_msgs.msg.MatrixFloat64()
      if self.trans_noise_cov is None:
        self.trans_noise_cov = argus_msgs.msg.MatrixFloat64()
      if self.prior_state_cov is None:
        self.prior_state_cov = argus_msgs.msg.MatrixFloat64()
      if self.post_state_cov is None:
        self.post_state_cov = argus_msgs.msg.MatrixFloat64()
    else:
      self.step_dt = 0.
      self.trans_jacobian = argus_msgs.msg.MatrixFloat64()
      self.trans_noise_cov = argus_msgs.msg.MatrixFloat64()
      self.prior_state_cov = argus_msgs.msg.MatrixFloat64()
      self.post_state_cov = argus_msgs.msg.MatrixFloat64()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_dB2I().pack(_x.step_dt, _x.trans_jacobian.column_major, _x.trans_jacobian.rows, _x.trans_jacobian.cols))
      length = len(self.trans_jacobian.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.trans_jacobian.data))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.trans_noise_cov.column_major, _x.trans_noise_cov.rows, _x.trans_noise_cov.cols))
      length = len(self.trans_noise_cov.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.trans_noise_cov.data))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.prior_state_cov.column_major, _x.prior_state_cov.rows, _x.prior_state_cov.cols))
      length = len(self.prior_state_cov.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.prior_state_cov.data))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.post_state_cov.column_major, _x.post_state_cov.rows, _x.post_state_cov.cols))
      length = len(self.post_state_cov.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.post_state_cov.data))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.trans_jacobian is None:
        self.trans_jacobian = argus_msgs.msg.MatrixFloat64()
      if self.trans_noise_cov is None:
        self.trans_noise_cov = argus_msgs.msg.MatrixFloat64()
      if self.prior_state_cov is None:
        self.prior_state_cov = argus_msgs.msg.MatrixFloat64()
      if self.post_state_cov is None:
        self.post_state_cov = argus_msgs.msg.MatrixFloat64()
      end = 0
      _x = self
      start = end
      end += 17
      (_x.step_dt, _x.trans_jacobian.column_major, _x.trans_jacobian.rows, _x.trans_jacobian.cols,) = _get_struct_dB2I().unpack(str[start:end])
      self.trans_jacobian.column_major = bool(self.trans_jacobian.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.trans_jacobian.data = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 9
      (_x.trans_noise_cov.column_major, _x.trans_noise_cov.rows, _x.trans_noise_cov.cols,) = _get_struct_B2I().unpack(str[start:end])
      self.trans_noise_cov.column_major = bool(self.trans_noise_cov.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.trans_noise_cov.data = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 9
      (_x.prior_state_cov.column_major, _x.prior_state_cov.rows, _x.prior_state_cov.cols,) = _get_struct_B2I().unpack(str[start:end])
      self.prior_state_cov.column_major = bool(self.prior_state_cov.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.prior_state_cov.data = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 9
      (_x.post_state_cov.column_major, _x.post_state_cov.rows, _x.post_state_cov.cols,) = _get_struct_B2I().unpack(str[start:end])
      self.post_state_cov.column_major = bool(self.post_state_cov.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.post_state_cov.data = struct.unpack(pattern, str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_dB2I().pack(_x.step_dt, _x.trans_jacobian.column_major, _x.trans_jacobian.rows, _x.trans_jacobian.cols))
      length = len(self.trans_jacobian.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.trans_jacobian.data.tostring())
      _x = self
      buff.write(_get_struct_B2I().pack(_x.trans_noise_cov.column_major, _x.trans_noise_cov.rows, _x.trans_noise_cov.cols))
      length = len(self.trans_noise_cov.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.trans_noise_cov.data.tostring())
      _x = self
      buff.write(_get_struct_B2I().pack(_x.prior_state_cov.column_major, _x.prior_state_cov.rows, _x.prior_state_cov.cols))
      length = len(self.prior_state_cov.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.prior_state_cov.data.tostring())
      _x = self
      buff.write(_get_struct_B2I().pack(_x.post_state_cov.column_major, _x.post_state_cov.rows, _x.post_state_cov.cols))
      length = len(self.post_state_cov.data)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.post_state_cov.data.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.trans_jacobian is None:
        self.trans_jacobian = argus_msgs.msg.MatrixFloat64()
      if self.trans_noise_cov is None:
        self.trans_noise_cov = argus_msgs.msg.MatrixFloat64()
      if self.prior_state_cov is None:
        self.prior_state_cov = argus_msgs.msg.MatrixFloat64()
      if self.post_state_cov is None:
        self.post_state_cov = argus_msgs.msg.MatrixFloat64()
      end = 0
      _x = self
      start = end
      end += 17
      (_x.step_dt, _x.trans_jacobian.column_major, _x.trans_jacobian.rows, _x.trans_jacobian.cols,) = _get_struct_dB2I().unpack(str[start:end])
      self.trans_jacobian.column_major = bool(self.trans_jacobian.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.trans_jacobian.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 9
      (_x.trans_noise_cov.column_major, _x.trans_noise_cov.rows, _x.trans_noise_cov.cols,) = _get_struct_B2I().unpack(str[start:end])
      self.trans_noise_cov.column_major = bool(self.trans_noise_cov.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.trans_noise_cov.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 9
      (_x.prior_state_cov.column_major, _x.prior_state_cov.rows, _x.prior_state_cov.cols,) = _get_struct_B2I().unpack(str[start:end])
      self.prior_state_cov.column_major = bool(self.prior_state_cov.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.prior_state_cov.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 9
      (_x.post_state_cov.column_major, _x.post_state_cov.rows, _x.post_state_cov.cols,) = _get_struct_B2I().unpack(str[start:end])
      self.post_state_cov.column_major = bool(self.post_state_cov.column_major)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.post_state_cov.data = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_dB2I = None
def _get_struct_dB2I():
    global _struct_dB2I
    if _struct_dB2I is None:
        _struct_dB2I = struct.Struct("<dB2I")
    return _struct_dB2I
_struct_B2I = None
def _get_struct_B2I():
    global _struct_B2I
    if _struct_B2I is None:
        _struct_B2I = struct.Struct("<B2I")
    return _struct_B2I
