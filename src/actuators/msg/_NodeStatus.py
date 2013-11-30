"""autogenerated by genpy from actuators/NodeStatus.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class NodeStatus(genpy.Message):
  _md5sum = "c876970d0bd9b81973de67a3c93e6445"
  _type = "actuators/NodeStatus"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """byte NODE_OK = 0
byte NODE_OFFLINE = 1

Header header

byte load_status
byte power_system_status
byte left_ballonet_status
byte right_ballonet_status
byte left_main_engine_servo_status
byte right_main_engine_servo_status
byte left_main_engine_status
byte right_main_engine_status
byte tail_electromotor_status
byte helium_valve_status
byte control_surface_status

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
  NODE_OK = 0
  NODE_OFFLINE = 1

  __slots__ = ['header','load_status','power_system_status','left_ballonet_status','right_ballonet_status','left_main_engine_servo_status','right_main_engine_servo_status','left_main_engine_status','right_main_engine_status','tail_electromotor_status','helium_valve_status','control_surface_status']
  _slot_types = ['std_msgs/Header','byte','byte','byte','byte','byte','byte','byte','byte','byte','byte','byte']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,load_status,power_system_status,left_ballonet_status,right_ballonet_status,left_main_engine_servo_status,right_main_engine_servo_status,left_main_engine_status,right_main_engine_status,tail_electromotor_status,helium_valve_status,control_surface_status

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(NodeStatus, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.load_status is None:
        self.load_status = 0
      if self.power_system_status is None:
        self.power_system_status = 0
      if self.left_ballonet_status is None:
        self.left_ballonet_status = 0
      if self.right_ballonet_status is None:
        self.right_ballonet_status = 0
      if self.left_main_engine_servo_status is None:
        self.left_main_engine_servo_status = 0
      if self.right_main_engine_servo_status is None:
        self.right_main_engine_servo_status = 0
      if self.left_main_engine_status is None:
        self.left_main_engine_status = 0
      if self.right_main_engine_status is None:
        self.right_main_engine_status = 0
      if self.tail_electromotor_status is None:
        self.tail_electromotor_status = 0
      if self.helium_valve_status is None:
        self.helium_valve_status = 0
      if self.control_surface_status is None:
        self.control_surface_status = 0
    else:
      self.header = std_msgs.msg.Header()
      self.load_status = 0
      self.power_system_status = 0
      self.left_ballonet_status = 0
      self.right_ballonet_status = 0
      self.left_main_engine_servo_status = 0
      self.right_main_engine_servo_status = 0
      self.left_main_engine_status = 0
      self.right_main_engine_status = 0
      self.tail_electromotor_status = 0
      self.helium_valve_status = 0
      self.control_surface_status = 0

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
      buff.write(_struct_11b.pack(_x.load_status, _x.power_system_status, _x.left_ballonet_status, _x.right_ballonet_status, _x.left_main_engine_servo_status, _x.right_main_engine_servo_status, _x.left_main_engine_status, _x.right_main_engine_status, _x.tail_electromotor_status, _x.helium_valve_status, _x.control_surface_status))
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
      end += 11
      (_x.load_status, _x.power_system_status, _x.left_ballonet_status, _x.right_ballonet_status, _x.left_main_engine_servo_status, _x.right_main_engine_servo_status, _x.left_main_engine_status, _x.right_main_engine_status, _x.tail_electromotor_status, _x.helium_valve_status, _x.control_surface_status,) = _struct_11b.unpack(str[start:end])
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
      buff.write(_struct_11b.pack(_x.load_status, _x.power_system_status, _x.left_ballonet_status, _x.right_ballonet_status, _x.left_main_engine_servo_status, _x.right_main_engine_servo_status, _x.left_main_engine_status, _x.right_main_engine_status, _x.tail_electromotor_status, _x.helium_valve_status, _x.control_surface_status))
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
      end += 11
      (_x.load_status, _x.power_system_status, _x.left_ballonet_status, _x.right_ballonet_status, _x.left_main_engine_servo_status, _x.right_main_engine_servo_status, _x.left_main_engine_status, _x.right_main_engine_status, _x.tail_electromotor_status, _x.helium_valve_status, _x.control_surface_status,) = _struct_11b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_11b = struct.Struct("<11b")