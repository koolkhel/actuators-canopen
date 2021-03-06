"""autogenerated by genpy from actuators_canopen/SetElectromotorsControlRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import actuators_canopen.msg
import std_msgs.msg

class SetElectromotorsControlRequest(genpy.Message):
  _md5sum = "0a0508ed7eb64fea55608a3264ccf1d5"
  _type = "actuators_canopen/SetElectromotorsControlRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """ElectromotorsState ctrl

================================================================================
MSG: actuators_canopen/ElectromotorsState
Header header

ElectromotorState left_em
ElectromotorState right_em
ElectromotorServoState left_em_servo
ElectromotorServoState right_em_servo
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: actuators_canopen/ElectromotorState
Header header

# 
float32 rate
================================================================================
MSG: actuators_canopen/ElectromotorServoState
Header header

# 
float32 anglex
float32 angley
#  
bool fix
"""
  __slots__ = ['ctrl']
  _slot_types = ['actuators_canopen/ElectromotorsState']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       ctrl

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetElectromotorsControlRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.ctrl is None:
        self.ctrl = actuators_canopen.msg.ElectromotorsState()
    else:
      self.ctrl = actuators_canopen.msg.ElectromotorsState()

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
      buff.write(_struct_3I.pack(_x.ctrl.header.seq, _x.ctrl.header.stamp.secs, _x.ctrl.header.stamp.nsecs))
      _x = self.ctrl.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.ctrl.left_em.header.seq, _x.ctrl.left_em.header.stamp.secs, _x.ctrl.left_em.header.stamp.nsecs))
      _x = self.ctrl.left_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.ctrl.left_em.rate, _x.ctrl.right_em.header.seq, _x.ctrl.right_em.header.stamp.secs, _x.ctrl.right_em.header.stamp.nsecs))
      _x = self.ctrl.right_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.ctrl.right_em.rate, _x.ctrl.left_em_servo.header.seq, _x.ctrl.left_em_servo.header.stamp.secs, _x.ctrl.left_em_servo.header.stamp.nsecs))
      _x = self.ctrl.left_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fB3I.pack(_x.ctrl.left_em_servo.anglex, _x.ctrl.left_em_servo.angley, _x.ctrl.left_em_servo.fix, _x.ctrl.right_em_servo.header.seq, _x.ctrl.right_em_servo.header.stamp.secs, _x.ctrl.right_em_servo.header.stamp.nsecs))
      _x = self.ctrl.right_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fB.pack(_x.ctrl.right_em_servo.anglex, _x.ctrl.right_em_servo.angley, _x.ctrl.right_em_servo.fix))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.ctrl is None:
        self.ctrl = actuators_canopen.msg.ElectromotorsState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.ctrl.header.seq, _x.ctrl.header.stamp.secs, _x.ctrl.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.ctrl.left_em.header.seq, _x.ctrl.left_em.header.stamp.secs, _x.ctrl.left_em.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.left_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.left_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.ctrl.left_em.rate, _x.ctrl.right_em.header.seq, _x.ctrl.right_em.header.stamp.secs, _x.ctrl.right_em.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.right_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.right_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.ctrl.right_em.rate, _x.ctrl.left_em_servo.header.seq, _x.ctrl.left_em_servo.header.stamp.secs, _x.ctrl.left_em_servo.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.left_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.left_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 21
      (_x.ctrl.left_em_servo.anglex, _x.ctrl.left_em_servo.angley, _x.ctrl.left_em_servo.fix, _x.ctrl.right_em_servo.header.seq, _x.ctrl.right_em_servo.header.stamp.secs, _x.ctrl.right_em_servo.header.stamp.nsecs,) = _struct_2fB3I.unpack(str[start:end])
      self.ctrl.left_em_servo.fix = bool(self.ctrl.left_em_servo.fix)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.right_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.right_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 9
      (_x.ctrl.right_em_servo.anglex, _x.ctrl.right_em_servo.angley, _x.ctrl.right_em_servo.fix,) = _struct_2fB.unpack(str[start:end])
      self.ctrl.right_em_servo.fix = bool(self.ctrl.right_em_servo.fix)
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
      buff.write(_struct_3I.pack(_x.ctrl.header.seq, _x.ctrl.header.stamp.secs, _x.ctrl.header.stamp.nsecs))
      _x = self.ctrl.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.ctrl.left_em.header.seq, _x.ctrl.left_em.header.stamp.secs, _x.ctrl.left_em.header.stamp.nsecs))
      _x = self.ctrl.left_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.ctrl.left_em.rate, _x.ctrl.right_em.header.seq, _x.ctrl.right_em.header.stamp.secs, _x.ctrl.right_em.header.stamp.nsecs))
      _x = self.ctrl.right_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.ctrl.right_em.rate, _x.ctrl.left_em_servo.header.seq, _x.ctrl.left_em_servo.header.stamp.secs, _x.ctrl.left_em_servo.header.stamp.nsecs))
      _x = self.ctrl.left_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fB3I.pack(_x.ctrl.left_em_servo.anglex, _x.ctrl.left_em_servo.angley, _x.ctrl.left_em_servo.fix, _x.ctrl.right_em_servo.header.seq, _x.ctrl.right_em_servo.header.stamp.secs, _x.ctrl.right_em_servo.header.stamp.nsecs))
      _x = self.ctrl.right_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fB.pack(_x.ctrl.right_em_servo.anglex, _x.ctrl.right_em_servo.angley, _x.ctrl.right_em_servo.fix))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.ctrl is None:
        self.ctrl = actuators_canopen.msg.ElectromotorsState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.ctrl.header.seq, _x.ctrl.header.stamp.secs, _x.ctrl.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.ctrl.left_em.header.seq, _x.ctrl.left_em.header.stamp.secs, _x.ctrl.left_em.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.left_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.left_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.ctrl.left_em.rate, _x.ctrl.right_em.header.seq, _x.ctrl.right_em.header.stamp.secs, _x.ctrl.right_em.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.right_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.right_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.ctrl.right_em.rate, _x.ctrl.left_em_servo.header.seq, _x.ctrl.left_em_servo.header.stamp.secs, _x.ctrl.left_em_servo.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.left_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.left_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 21
      (_x.ctrl.left_em_servo.anglex, _x.ctrl.left_em_servo.angley, _x.ctrl.left_em_servo.fix, _x.ctrl.right_em_servo.header.seq, _x.ctrl.right_em_servo.header.stamp.secs, _x.ctrl.right_em_servo.header.stamp.nsecs,) = _struct_2fB3I.unpack(str[start:end])
      self.ctrl.left_em_servo.fix = bool(self.ctrl.left_em_servo.fix)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ctrl.right_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.ctrl.right_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 9
      (_x.ctrl.right_em_servo.anglex, _x.ctrl.right_em_servo.angley, _x.ctrl.right_em_servo.fix,) = _struct_2fB.unpack(str[start:end])
      self.ctrl.right_em_servo.fix = bool(self.ctrl.right_em_servo.fix)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_f3I = struct.Struct("<f3I")
_struct_3I = struct.Struct("<3I")
_struct_2fB3I = struct.Struct("<2fB3I")
_struct_2fB = struct.Struct("<2fB")
"""autogenerated by genpy from actuators_canopen/SetElectromotorsControlResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetElectromotorsControlResponse(genpy.Message):
  _md5sum = "25458147911545c320c4c0a299eff763"
  _type = "actuators_canopen/SetElectromotorsControlResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 result

"""
  __slots__ = ['result']
  _slot_types = ['uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetElectromotorsControlResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = 0
    else:
      self.result = 0

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
      buff.write(_struct_B.pack(self.result))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _struct_B.unpack(str[start:end])
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
      buff.write(_struct_B.pack(self.result))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _struct_B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class SetElectromotorsControl(object):
  _type          = 'actuators_canopen/SetElectromotorsControl'
  _md5sum = '2b3f3cb6678cc01e00002b2bb1498b93'
  _request_class  = SetElectromotorsControlRequest
  _response_class = SetElectromotorsControlResponse
