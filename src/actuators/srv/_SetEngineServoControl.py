"""autogenerated by genpy from actuators/SetEngineServoControlRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import actuators.msg
import std_msgs.msg

class SetEngineServoControlRequest(genpy.Message):
  _md5sum = "14dba8c0ec8cd4dc91b9bcb43f934462"
  _type = "actuators/SetEngineServoControlRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """EngineServoState ctrl

================================================================================
MSG: actuators/EngineServoState
Header header

# 
float32 angle
#  
bool fix
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

"""
  __slots__ = ['ctrl']
  _slot_types = ['actuators/EngineServoState']

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
      super(SetEngineServoControlRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.ctrl is None:
        self.ctrl = actuators.msg.EngineServoState()
    else:
      self.ctrl = actuators.msg.EngineServoState()

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
      buff.write(_struct_fB.pack(_x.ctrl.angle, _x.ctrl.fix))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.ctrl is None:
        self.ctrl = actuators.msg.EngineServoState()
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
      end += 5
      (_x.ctrl.angle, _x.ctrl.fix,) = _struct_fB.unpack(str[start:end])
      self.ctrl.fix = bool(self.ctrl.fix)
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
      buff.write(_struct_fB.pack(_x.ctrl.angle, _x.ctrl.fix))
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
        self.ctrl = actuators.msg.EngineServoState()
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
      end += 5
      (_x.ctrl.angle, _x.ctrl.fix,) = _struct_fB.unpack(str[start:end])
      self.ctrl.fix = bool(self.ctrl.fix)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_fB = struct.Struct("<fB")
"""autogenerated by genpy from actuators/SetEngineServoControlResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetEngineServoControlResponse(genpy.Message):
  _md5sum = "25458147911545c320c4c0a299eff763"
  _type = "actuators/SetEngineServoControlResponse"
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
      super(SetEngineServoControlResponse, self).__init__(*args, **kwds)
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
class SetEngineServoControl(object):
  _type          = 'actuators/SetEngineServoControl'
  _md5sum = '0a373022f0310157ad7e547ce7c01f15'
  _request_class  = SetEngineServoControlRequest
  _response_class = SetEngineServoControlResponse
