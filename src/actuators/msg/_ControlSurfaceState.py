"""autogenerated by genpy from actuators/ControlSurfaceState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class ControlSurfaceState(genpy.Message):
  _md5sum = "fbb29f54884597cd007967851cd75ca7"
  _type = "actuators/ControlSurfaceState"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

float32 left_control_surface_X
float32 left_control_surface_Y
float32 right_control_surface_X
float32 right_control_surface_Y

float32 control_surface_1_current
float32 control_surface_2_current
float32 control_surface_3_current
float32 control_surface_4_current
float32 control_surface_5_current
float32 control_surface_6_current
float32 control_surface_7_current
float32 control_surface_8_current


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
  __slots__ = ['header','left_control_surface_X','left_control_surface_Y','right_control_surface_X','right_control_surface_Y','control_surface_1_current','control_surface_2_current','control_surface_3_current','control_surface_4_current','control_surface_5_current','control_surface_6_current','control_surface_7_current','control_surface_8_current']
  _slot_types = ['std_msgs/Header','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,left_control_surface_X,left_control_surface_Y,right_control_surface_X,right_control_surface_Y,control_surface_1_current,control_surface_2_current,control_surface_3_current,control_surface_4_current,control_surface_5_current,control_surface_6_current,control_surface_7_current,control_surface_8_current

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ControlSurfaceState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.left_control_surface_X is None:
        self.left_control_surface_X = 0.
      if self.left_control_surface_Y is None:
        self.left_control_surface_Y = 0.
      if self.right_control_surface_X is None:
        self.right_control_surface_X = 0.
      if self.right_control_surface_Y is None:
        self.right_control_surface_Y = 0.
      if self.control_surface_1_current is None:
        self.control_surface_1_current = 0.
      if self.control_surface_2_current is None:
        self.control_surface_2_current = 0.
      if self.control_surface_3_current is None:
        self.control_surface_3_current = 0.
      if self.control_surface_4_current is None:
        self.control_surface_4_current = 0.
      if self.control_surface_5_current is None:
        self.control_surface_5_current = 0.
      if self.control_surface_6_current is None:
        self.control_surface_6_current = 0.
      if self.control_surface_7_current is None:
        self.control_surface_7_current = 0.
      if self.control_surface_8_current is None:
        self.control_surface_8_current = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.left_control_surface_X = 0.
      self.left_control_surface_Y = 0.
      self.right_control_surface_X = 0.
      self.right_control_surface_Y = 0.
      self.control_surface_1_current = 0.
      self.control_surface_2_current = 0.
      self.control_surface_3_current = 0.
      self.control_surface_4_current = 0.
      self.control_surface_5_current = 0.
      self.control_surface_6_current = 0.
      self.control_surface_7_current = 0.
      self.control_surface_8_current = 0.

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
      buff.write(_struct_12f.pack(_x.left_control_surface_X, _x.left_control_surface_Y, _x.right_control_surface_X, _x.right_control_surface_Y, _x.control_surface_1_current, _x.control_surface_2_current, _x.control_surface_3_current, _x.control_surface_4_current, _x.control_surface_5_current, _x.control_surface_6_current, _x.control_surface_7_current, _x.control_surface_8_current))
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
      end += 48
      (_x.left_control_surface_X, _x.left_control_surface_Y, _x.right_control_surface_X, _x.right_control_surface_Y, _x.control_surface_1_current, _x.control_surface_2_current, _x.control_surface_3_current, _x.control_surface_4_current, _x.control_surface_5_current, _x.control_surface_6_current, _x.control_surface_7_current, _x.control_surface_8_current,) = _struct_12f.unpack(str[start:end])
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
      buff.write(_struct_12f.pack(_x.left_control_surface_X, _x.left_control_surface_Y, _x.right_control_surface_X, _x.right_control_surface_Y, _x.control_surface_1_current, _x.control_surface_2_current, _x.control_surface_3_current, _x.control_surface_4_current, _x.control_surface_5_current, _x.control_surface_6_current, _x.control_surface_7_current, _x.control_surface_8_current))
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
      end += 48
      (_x.left_control_surface_X, _x.left_control_surface_Y, _x.right_control_surface_X, _x.right_control_surface_Y, _x.control_surface_1_current, _x.control_surface_2_current, _x.control_surface_3_current, _x.control_surface_4_current, _x.control_surface_5_current, _x.control_surface_6_current, _x.control_surface_7_current, _x.control_surface_8_current,) = _struct_12f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_12f = struct.Struct("<12f")