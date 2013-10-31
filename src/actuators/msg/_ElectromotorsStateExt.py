"""autogenerated by genpy from actuators/ElectromotorsStateExt.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import actuators.msg
import std_msgs.msg

class ElectromotorsStateExt(genpy.Message):
  _md5sum = "14b11ded2ebed98599d9eb9bfddc0be9"
  _type = "actuators/ElectromotorsStateExt"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

ElectromotorsState basic_state

uint16 bus_voltage_left # bus voltage of the left el.dv.
uint8 bus_current_left # electric current bus left el.dv.
uint8 temperature_radiator_driver_left # radiator temperature of the driver left el.dv.
uint8 temperature_shell_left # Kozhukhovo the temperature of the left el.dv.
uint16 bus_voltage_right # bus voltage right el.dv.
uint8 bus_current_right # electric current bus right el.dv.
uint8 temperature_radiator_driver_right # radiator temperature of the driver right el.dv.
uint8 temperature_shell_right # Kozhukhovo right temperature el.dv.
uint8 errors_cnt # Number of Accidents (1 byte)
uint8 errors_flag1 # high byte code crashes (1 byte)
uint8 errors_flag2 # low byte code crashes (1 byte)
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
MSG: actuators/ElectromotorsState
Header header

ElectromotorState left_em
ElectromotorState right_em
ElectromotorServoState left_em_servo
ElectromotorServoState right_em_servo
================================================================================
MSG: actuators/ElectromotorState
Header header

# 
float32 rate
================================================================================
MSG: actuators/ElectromotorServoState
Header header

# 
float32 anglex
float32 angley
#  
bool fix
"""
  __slots__ = ['header','basic_state','bus_voltage_left','bus_current_left','temperature_radiator_driver_left','temperature_shell_left','bus_voltage_right','bus_current_right','temperature_radiator_driver_right','temperature_shell_right','errors_cnt','errors_flag1','errors_flag2']
  _slot_types = ['std_msgs/Header','actuators/ElectromotorsState','uint16','uint8','uint8','uint8','uint16','uint8','uint8','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,basic_state,bus_voltage_left,bus_current_left,temperature_radiator_driver_left,temperature_shell_left,bus_voltage_right,bus_current_right,temperature_radiator_driver_right,temperature_shell_right,errors_cnt,errors_flag1,errors_flag2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ElectromotorsStateExt, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.basic_state is None:
        self.basic_state = actuators.msg.ElectromotorsState()
      if self.bus_voltage_left is None:
        self.bus_voltage_left = 0
      if self.bus_current_left is None:
        self.bus_current_left = 0
      if self.temperature_radiator_driver_left is None:
        self.temperature_radiator_driver_left = 0
      if self.temperature_shell_left is None:
        self.temperature_shell_left = 0
      if self.bus_voltage_right is None:
        self.bus_voltage_right = 0
      if self.bus_current_right is None:
        self.bus_current_right = 0
      if self.temperature_radiator_driver_right is None:
        self.temperature_radiator_driver_right = 0
      if self.temperature_shell_right is None:
        self.temperature_shell_right = 0
      if self.errors_cnt is None:
        self.errors_cnt = 0
      if self.errors_flag1 is None:
        self.errors_flag1 = 0
      if self.errors_flag2 is None:
        self.errors_flag2 = 0
    else:
      self.header = std_msgs.msg.Header()
      self.basic_state = actuators.msg.ElectromotorsState()
      self.bus_voltage_left = 0
      self.bus_current_left = 0
      self.temperature_radiator_driver_left = 0
      self.temperature_shell_left = 0
      self.bus_voltage_right = 0
      self.bus_current_right = 0
      self.temperature_radiator_driver_right = 0
      self.temperature_shell_right = 0
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
      buff.write(_struct_3I.pack(_x.basic_state.left_em.header.seq, _x.basic_state.left_em.header.stamp.secs, _x.basic_state.left_em.header.stamp.nsecs))
      _x = self.basic_state.left_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.basic_state.left_em.rate, _x.basic_state.right_em.header.seq, _x.basic_state.right_em.header.stamp.secs, _x.basic_state.right_em.header.stamp.nsecs))
      _x = self.basic_state.right_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.basic_state.right_em.rate, _x.basic_state.left_em_servo.header.seq, _x.basic_state.left_em_servo.header.stamp.secs, _x.basic_state.left_em_servo.header.stamp.nsecs))
      _x = self.basic_state.left_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fB3I.pack(_x.basic_state.left_em_servo.anglex, _x.basic_state.left_em_servo.angley, _x.basic_state.left_em_servo.fix, _x.basic_state.right_em_servo.header.seq, _x.basic_state.right_em_servo.header.stamp.secs, _x.basic_state.right_em_servo.header.stamp.nsecs))
      _x = self.basic_state.right_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fBH3BH6B.pack(_x.basic_state.right_em_servo.anglex, _x.basic_state.right_em_servo.angley, _x.basic_state.right_em_servo.fix, _x.bus_voltage_left, _x.bus_current_left, _x.temperature_radiator_driver_left, _x.temperature_shell_left, _x.bus_voltage_right, _x.bus_current_right, _x.temperature_radiator_driver_right, _x.temperature_shell_right, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2))
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
        self.basic_state = actuators.msg.ElectromotorsState()
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
      end += 12
      (_x.basic_state.left_em.header.seq, _x.basic_state.left_em.header.stamp.secs, _x.basic_state.left_em.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.left_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.left_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.basic_state.left_em.rate, _x.basic_state.right_em.header.seq, _x.basic_state.right_em.header.stamp.secs, _x.basic_state.right_em.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.right_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.right_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.basic_state.right_em.rate, _x.basic_state.left_em_servo.header.seq, _x.basic_state.left_em_servo.header.stamp.secs, _x.basic_state.left_em_servo.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.left_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.left_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 21
      (_x.basic_state.left_em_servo.anglex, _x.basic_state.left_em_servo.angley, _x.basic_state.left_em_servo.fix, _x.basic_state.right_em_servo.header.seq, _x.basic_state.right_em_servo.header.stamp.secs, _x.basic_state.right_em_servo.header.stamp.nsecs,) = _struct_2fB3I.unpack(str[start:end])
      self.basic_state.left_em_servo.fix = bool(self.basic_state.left_em_servo.fix)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.right_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.right_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 22
      (_x.basic_state.right_em_servo.anglex, _x.basic_state.right_em_servo.angley, _x.basic_state.right_em_servo.fix, _x.bus_voltage_left, _x.bus_current_left, _x.temperature_radiator_driver_left, _x.temperature_shell_left, _x.bus_voltage_right, _x.bus_current_right, _x.temperature_radiator_driver_right, _x.temperature_shell_right, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2,) = _struct_2fBH3BH6B.unpack(str[start:end])
      self.basic_state.right_em_servo.fix = bool(self.basic_state.right_em_servo.fix)
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
      buff.write(_struct_3I.pack(_x.basic_state.left_em.header.seq, _x.basic_state.left_em.header.stamp.secs, _x.basic_state.left_em.header.stamp.nsecs))
      _x = self.basic_state.left_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.basic_state.left_em.rate, _x.basic_state.right_em.header.seq, _x.basic_state.right_em.header.stamp.secs, _x.basic_state.right_em.header.stamp.nsecs))
      _x = self.basic_state.right_em.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_f3I.pack(_x.basic_state.right_em.rate, _x.basic_state.left_em_servo.header.seq, _x.basic_state.left_em_servo.header.stamp.secs, _x.basic_state.left_em_servo.header.stamp.nsecs))
      _x = self.basic_state.left_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fB3I.pack(_x.basic_state.left_em_servo.anglex, _x.basic_state.left_em_servo.angley, _x.basic_state.left_em_servo.fix, _x.basic_state.right_em_servo.header.seq, _x.basic_state.right_em_servo.header.stamp.secs, _x.basic_state.right_em_servo.header.stamp.nsecs))
      _x = self.basic_state.right_em_servo.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2fBH3BH6B.pack(_x.basic_state.right_em_servo.anglex, _x.basic_state.right_em_servo.angley, _x.basic_state.right_em_servo.fix, _x.bus_voltage_left, _x.bus_current_left, _x.temperature_radiator_driver_left, _x.temperature_shell_left, _x.bus_voltage_right, _x.bus_current_right, _x.temperature_radiator_driver_right, _x.temperature_shell_right, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2))
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
        self.basic_state = actuators.msg.ElectromotorsState()
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
      end += 12
      (_x.basic_state.left_em.header.seq, _x.basic_state.left_em.header.stamp.secs, _x.basic_state.left_em.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.left_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.left_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.basic_state.left_em.rate, _x.basic_state.right_em.header.seq, _x.basic_state.right_em.header.stamp.secs, _x.basic_state.right_em.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.right_em.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.right_em.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.basic_state.right_em.rate, _x.basic_state.left_em_servo.header.seq, _x.basic_state.left_em_servo.header.stamp.secs, _x.basic_state.left_em_servo.header.stamp.nsecs,) = _struct_f3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.left_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.left_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 21
      (_x.basic_state.left_em_servo.anglex, _x.basic_state.left_em_servo.angley, _x.basic_state.left_em_servo.fix, _x.basic_state.right_em_servo.header.seq, _x.basic_state.right_em_servo.header.stamp.secs, _x.basic_state.right_em_servo.header.stamp.nsecs,) = _struct_2fB3I.unpack(str[start:end])
      self.basic_state.left_em_servo.fix = bool(self.basic_state.left_em_servo.fix)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.basic_state.right_em_servo.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.basic_state.right_em_servo.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 22
      (_x.basic_state.right_em_servo.anglex, _x.basic_state.right_em_servo.angley, _x.basic_state.right_em_servo.fix, _x.bus_voltage_left, _x.bus_current_left, _x.temperature_radiator_driver_left, _x.temperature_shell_left, _x.bus_voltage_right, _x.bus_current_right, _x.temperature_radiator_driver_right, _x.temperature_shell_right, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2,) = _struct_2fBH3BH6B.unpack(str[start:end])
      self.basic_state.right_em_servo.fix = bool(self.basic_state.right_em_servo.fix)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_f3I = struct.Struct("<f3I")
_struct_3I = struct.Struct("<3I")
_struct_2fB3I = struct.Struct("<2fB3I")
_struct_2fBH3BH6B = struct.Struct("<2fBH3BH6B")
