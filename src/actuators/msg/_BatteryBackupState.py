"""autogenerated by genpy from actuators/BatteryBackupState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class BatteryBackupState(genpy.Message):
  _md5sum = "d5cf22838b366b85c307db39de59d020"
  _type = "actuators/BatteryBackupState"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

byte FAILURE_FLAG_OK = 0
byte FAILURE_FLAG_FAILURE = 1

byte FAILURE_CODE_OK = 0
byte FAILURE_CODE_CHARGE_ONLY = 1
byte FAILURE_CODE_DISCHARGE_ONLY = 2
byte FAILURE_CODE_FAILURE = 3

uint8 battery1_failure_flag
uint8 battery1_failure_code

uint8 battery2_failure_flag
uint8 battery2_failure_code

uint8 battery3_failure_flag
uint8 battery3_failure_code

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
  # Pseudo-constants
  FAILURE_FLAG_OK = 0
  FAILURE_FLAG_FAILURE = 1
  FAILURE_CODE_OK = 0
  FAILURE_CODE_CHARGE_ONLY = 1
  FAILURE_CODE_DISCHARGE_ONLY = 2
  FAILURE_CODE_FAILURE = 3

  __slots__ = ['header','battery1_failure_flag','battery1_failure_code','battery2_failure_flag','battery2_failure_code','battery3_failure_flag','battery3_failure_code']
  _slot_types = ['std_msgs/Header','uint8','uint8','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,battery1_failure_flag,battery1_failure_code,battery2_failure_flag,battery2_failure_code,battery3_failure_flag,battery3_failure_code

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BatteryBackupState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.battery1_failure_flag is None:
        self.battery1_failure_flag = 0
      if self.battery1_failure_code is None:
        self.battery1_failure_code = 0
      if self.battery2_failure_flag is None:
        self.battery2_failure_flag = 0
      if self.battery2_failure_code is None:
        self.battery2_failure_code = 0
      if self.battery3_failure_flag is None:
        self.battery3_failure_flag = 0
      if self.battery3_failure_code is None:
        self.battery3_failure_code = 0
    else:
      self.header = std_msgs.msg.Header()
      self.battery1_failure_flag = 0
      self.battery1_failure_code = 0
      self.battery2_failure_flag = 0
      self.battery2_failure_code = 0
      self.battery3_failure_flag = 0
      self.battery3_failure_code = 0

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
      buff.write(_struct_6B.pack(_x.battery1_failure_flag, _x.battery1_failure_code, _x.battery2_failure_flag, _x.battery2_failure_code, _x.battery3_failure_flag, _x.battery3_failure_code))
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
      end += 6
      (_x.battery1_failure_flag, _x.battery1_failure_code, _x.battery2_failure_flag, _x.battery2_failure_code, _x.battery3_failure_flag, _x.battery3_failure_code,) = _struct_6B.unpack(str[start:end])
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
      buff.write(_struct_6B.pack(_x.battery1_failure_flag, _x.battery1_failure_code, _x.battery2_failure_flag, _x.battery2_failure_code, _x.battery3_failure_flag, _x.battery3_failure_code))
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
      end += 6
      (_x.battery1_failure_flag, _x.battery1_failure_code, _x.battery2_failure_flag, _x.battery2_failure_code, _x.battery3_failure_flag, _x.battery3_failure_code,) = _struct_6B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_6B = struct.Struct("<6B")
