"""autogenerated by genpy from actuators_canopen/EngineServoStateExt.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import actuators_canopen.msg
import std_msgs.msg

class EngineServoStateExt(genpy.Message):
  _md5sum = "9fdb46ee33952513c7b16eb75b5bc99f"
  _type = "actuators_canopen/EngineServoStateExt"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

EngineServoState basic_state

uint8 errors_cnt # of accidents
uint8 errors_flag1 # error code 
uint8 errors_flag2 # error code 

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
MSG: actuators_canopen/EngineServoState
Header header

# 
float32 angle
#  
bool fix
"""
  __slots__ = ['header','basic_state','errors_cnt','errors_flag1','errors_flag2']
  _slot_types = ['std_msgs/Header','actuators_canopen/EngineServoState','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,basic_state,errors_cnt,errors_flag1,errors_flag2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(EngineServoStateExt, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.basic_state is None:
        self.basic_state = actuators_canopen.msg.EngineServoState()
      if self.errors_cnt is None:
        self.errors_cnt = 0
      if self.errors_flag1 is None:
        self.errors_flag1 = 0
      if self.errors_flag2 is None:
        self.errors_flag2 = 0
    else:
      self.header = std_msgs.msg.Header()
      self.basic_state = actuators_canopen.msg.EngineServoState()
      self.errors_cnt = 0
      self.errors_flag1 = 0
      self.errors_flag2 = 0

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.basic_state.header.seq, _x.basic_state.header.stamp.secs, _x.basic_state.header.stamp.nsecs))
      _x = self.basic_state.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f4B.pack(_x.basic_state.angle, _x.basic_state.fix, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.basic_state is None:
        self.basic_state = actuators_canopen.msg.EngineServoState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.basic_state.header.seq, _x.basic_state.header.stamp.secs, _x.basic_state.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.basic_state.angle, _x.basic_state.fix, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2,) = _struct_f4B.unpack(str[start:end])
      self.basic_state.fix = bool(self.basic_state.fix)
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.basic_state.header.seq, _x.basic_state.header.stamp.secs, _x.basic_state.header.stamp.nsecs))
      _x = self.basic_state.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f4B.pack(_x.basic_state.angle, _x.basic_state.fix, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.basic_state is None:
        self.basic_state = actuators_canopen.msg.EngineServoState()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.basic_state.header.seq, _x.basic_state.header.stamp.secs, _x.basic_state.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.basic_state.angle, _x.basic_state.fix, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2,) = _struct_f4B.unpack(str[start:end])
      self.basic_state.fix = bool(self.basic_state.fix)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_f4B = struct.Struct("<f4B")
