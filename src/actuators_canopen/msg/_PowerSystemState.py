"""autogenerated by genpy from actuators_canopen/PowerSystemState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class PowerSystemState(genpy.Message):
  _md5sum = "069c456d8916b3e1494e50bca3b115b4"
  _type = "actuators_canopen/PowerSystemState"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

uint8 left_electromotor_current # Current left motor
uint16 left_electromotor_voltage # voltage of the left motor
uint8 left_electromotor_temperature # temperature of the left motor
uint16 left_electromotor_rate # speed of rotation of the left motor
uint8 left_electromotor_regime # mode of movement of the left motor
uint8 right_electromotor_current # Current right motor
uint16 right_electromotor_voltage # right motor voltage
uint8 right_electromotor_temperature # temperature of the right motor
uint16 right_electromotor_rate # speed of the right motor
uint8 right_electromotor_regime # mode of movement of the right motor
uint8 accumulator1_current # current a battery
uint16 accumulator1_voltage # 1 battery voltage
uint8 accumulator1_charge # akkusulyatora a quantity of electricity
uint8 accumulator1_temperature # 1 battery temperature
uint8 accumulator1_regime # 1 battery operation mode
uint8 accumulator2_current # 2 battery current
uint16 accumulator2_voltage # 2 battery voltage
uint8 accumulator2_charge # Number of battery power 2
uint8 accumulator2_temperature # 2 battery temperature
uint8 accumulator2_regime # Battery operation mode 2
uint8 accumulator3_current # 3 battery current
uint16 accumulator3_voltage # 3 battery voltage
uint8 accumulator3_charge # 3 Number of battery power
uint8 accumulator3_temperature # 3 battery temperature
uint8 accumulator3_regime # 3 battery operation mode
uint8 accumulator4_current # 4 battery current
uint16 accumulator4_voltage # 4 battery voltage
uint8 accumulator4_charge # The amount of electricity the battery 4
uint8 accumulator4_temperature # 4 battery temperature
uint8 accumulator4_regime # 4 battery operation mode
uint8 left_electromotor_start_current # Current left the motor to start
uint8 right_electromotor_start_current # Current right motor to start
uint8 left_pump1_current # operating current of the left ventilyatora1
uint8 left_pump2_current # operating current of the left ventilyatora2
uint8 right_pump1_current # current working right ventilyatora1
uint8 right_pump2_current # current working right ventilyatora2
uint8 left_electromotor_operating_current # operating current of the left tail of the electric motor
uint8 right_electromotor_operating_current # operating current of the right tail of the electric motor
uint8 left_electromotor_servo_operating_current # operating current servo motor rotational unit of the caudal
uint8 left_engine_servo_operating_current # operating current servo rotary device of the left main engine
uint8 left_engine_servo_regime # operating mode servo rotary device of the left main engine
uint8 right_engine_servo_operating_current # operating current servo rotary device of the right main engine
uint8 right_engine_servo_regime # operating mode servo rotary device of the right main engine
uint8 load1_operating_current # backup load operating current of an equipment
uint8 load1_regime # backup load operating conditions of equipment 1
uint8 load2_operating_current # backup load operating current equipment2
uint8 load2_regime # backup load operating conditions of equipment 2
uint8 load3_operating_current # standby operating current load equipment 3
uint8 load3_regime # backup load operating conditions of equipment 3
uint8 load4_operating_current # backup load operating current of equipment 4
uint8 load4_regime # backup load operating conditions of equipment 4
uint8 load5_operating_current # standby operating current load equipment 5
uint8 load5_regime # backup load operating conditions of equipment 5
uint8 bus28v1_voltage # Voltage 28Vshiny1
uint8 bus28v2_voltage # Voltage 28Vshiny2
uint8 bus28v3_voltage # Voltage 28Vshiny3
uint8 bus28v4_voltage # Voltage 28Vshiny4
uint16 bus270v1_voltage # Voltage 270Vshiny1
uint16 bus270v2_voltage # Voltage 270Vshiny2
uint16 bus270v3_voltage # Voltage 270Vshiny3
uint16 bus270v4_voltage # Voltage 270Vshiny4
uint8 aux_accumulator_regime # operating mode on-board secondary battery
uint16 switch_state_left_power_module # of the switch equipment left power bay
uint16 switch_state_load # the switch hardware load kobinata
uint16 switch_state_right_power_module # of the switch of the power of the right equipment bay
uint16 reserve # booking
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

"""
  __slots__ = ['header','left_electromotor_current','left_electromotor_voltage','left_electromotor_temperature','left_electromotor_rate','left_electromotor_regime','right_electromotor_current','right_electromotor_voltage','right_electromotor_temperature','right_electromotor_rate','right_electromotor_regime','accumulator1_current','accumulator1_voltage','accumulator1_charge','accumulator1_temperature','accumulator1_regime','accumulator2_current','accumulator2_voltage','accumulator2_charge','accumulator2_temperature','accumulator2_regime','accumulator3_current','accumulator3_voltage','accumulator3_charge','accumulator3_temperature','accumulator3_regime','accumulator4_current','accumulator4_voltage','accumulator4_charge','accumulator4_temperature','accumulator4_regime','left_electromotor_start_current','right_electromotor_start_current','left_pump1_current','left_pump2_current','right_pump1_current','right_pump2_current','left_electromotor_operating_current','right_electromotor_operating_current','left_electromotor_servo_operating_current','left_engine_servo_operating_current','left_engine_servo_regime','right_engine_servo_operating_current','right_engine_servo_regime','load1_operating_current','load1_regime','load2_operating_current','load2_regime','load3_operating_current','load3_regime','load4_operating_current','load4_regime','load5_operating_current','load5_regime','bus28v1_voltage','bus28v2_voltage','bus28v3_voltage','bus28v4_voltage','bus270v1_voltage','bus270v2_voltage','bus270v3_voltage','bus270v4_voltage','aux_accumulator_regime','switch_state_left_power_module','switch_state_load','switch_state_right_power_module','reserve','errors_cnt','errors_flag1','errors_flag2']
  _slot_types = ['std_msgs/Header','uint8','uint16','uint8','uint16','uint8','uint8','uint16','uint8','uint16','uint8','uint8','uint16','uint8','uint8','uint8','uint8','uint16','uint8','uint8','uint8','uint8','uint16','uint8','uint8','uint8','uint8','uint16','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint16','uint16','uint16','uint16','uint8','uint16','uint16','uint16','uint16','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,left_electromotor_current,left_electromotor_voltage,left_electromotor_temperature,left_electromotor_rate,left_electromotor_regime,right_electromotor_current,right_electromotor_voltage,right_electromotor_temperature,right_electromotor_rate,right_electromotor_regime,accumulator1_current,accumulator1_voltage,accumulator1_charge,accumulator1_temperature,accumulator1_regime,accumulator2_current,accumulator2_voltage,accumulator2_charge,accumulator2_temperature,accumulator2_regime,accumulator3_current,accumulator3_voltage,accumulator3_charge,accumulator3_temperature,accumulator3_regime,accumulator4_current,accumulator4_voltage,accumulator4_charge,accumulator4_temperature,accumulator4_regime,left_electromotor_start_current,right_electromotor_start_current,left_pump1_current,left_pump2_current,right_pump1_current,right_pump2_current,left_electromotor_operating_current,right_electromotor_operating_current,left_electromotor_servo_operating_current,left_engine_servo_operating_current,left_engine_servo_regime,right_engine_servo_operating_current,right_engine_servo_regime,load1_operating_current,load1_regime,load2_operating_current,load2_regime,load3_operating_current,load3_regime,load4_operating_current,load4_regime,load5_operating_current,load5_regime,bus28v1_voltage,bus28v2_voltage,bus28v3_voltage,bus28v4_voltage,bus270v1_voltage,bus270v2_voltage,bus270v3_voltage,bus270v4_voltage,aux_accumulator_regime,switch_state_left_power_module,switch_state_load,switch_state_right_power_module,reserve,errors_cnt,errors_flag1,errors_flag2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PowerSystemState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.left_electromotor_current is None:
        self.left_electromotor_current = 0
      if self.left_electromotor_voltage is None:
        self.left_electromotor_voltage = 0
      if self.left_electromotor_temperature is None:
        self.left_electromotor_temperature = 0
      if self.left_electromotor_rate is None:
        self.left_electromotor_rate = 0
      if self.left_electromotor_regime is None:
        self.left_electromotor_regime = 0
      if self.right_electromotor_current is None:
        self.right_electromotor_current = 0
      if self.right_electromotor_voltage is None:
        self.right_electromotor_voltage = 0
      if self.right_electromotor_temperature is None:
        self.right_electromotor_temperature = 0
      if self.right_electromotor_rate is None:
        self.right_electromotor_rate = 0
      if self.right_electromotor_regime is None:
        self.right_electromotor_regime = 0
      if self.accumulator1_current is None:
        self.accumulator1_current = 0
      if self.accumulator1_voltage is None:
        self.accumulator1_voltage = 0
      if self.accumulator1_charge is None:
        self.accumulator1_charge = 0
      if self.accumulator1_temperature is None:
        self.accumulator1_temperature = 0
      if self.accumulator1_regime is None:
        self.accumulator1_regime = 0
      if self.accumulator2_current is None:
        self.accumulator2_current = 0
      if self.accumulator2_voltage is None:
        self.accumulator2_voltage = 0
      if self.accumulator2_charge is None:
        self.accumulator2_charge = 0
      if self.accumulator2_temperature is None:
        self.accumulator2_temperature = 0
      if self.accumulator2_regime is None:
        self.accumulator2_regime = 0
      if self.accumulator3_current is None:
        self.accumulator3_current = 0
      if self.accumulator3_voltage is None:
        self.accumulator3_voltage = 0
      if self.accumulator3_charge is None:
        self.accumulator3_charge = 0
      if self.accumulator3_temperature is None:
        self.accumulator3_temperature = 0
      if self.accumulator3_regime is None:
        self.accumulator3_regime = 0
      if self.accumulator4_current is None:
        self.accumulator4_current = 0
      if self.accumulator4_voltage is None:
        self.accumulator4_voltage = 0
      if self.accumulator4_charge is None:
        self.accumulator4_charge = 0
      if self.accumulator4_temperature is None:
        self.accumulator4_temperature = 0
      if self.accumulator4_regime is None:
        self.accumulator4_regime = 0
      if self.left_electromotor_start_current is None:
        self.left_electromotor_start_current = 0
      if self.right_electromotor_start_current is None:
        self.right_electromotor_start_current = 0
      if self.left_pump1_current is None:
        self.left_pump1_current = 0
      if self.left_pump2_current is None:
        self.left_pump2_current = 0
      if self.right_pump1_current is None:
        self.right_pump1_current = 0
      if self.right_pump2_current is None:
        self.right_pump2_current = 0
      if self.left_electromotor_operating_current is None:
        self.left_electromotor_operating_current = 0
      if self.right_electromotor_operating_current is None:
        self.right_electromotor_operating_current = 0
      if self.left_electromotor_servo_operating_current is None:
        self.left_electromotor_servo_operating_current = 0
      if self.left_engine_servo_operating_current is None:
        self.left_engine_servo_operating_current = 0
      if self.left_engine_servo_regime is None:
        self.left_engine_servo_regime = 0
      if self.right_engine_servo_operating_current is None:
        self.right_engine_servo_operating_current = 0
      if self.right_engine_servo_regime is None:
        self.right_engine_servo_regime = 0
      if self.load1_operating_current is None:
        self.load1_operating_current = 0
      if self.load1_regime is None:
        self.load1_regime = 0
      if self.load2_operating_current is None:
        self.load2_operating_current = 0
      if self.load2_regime is None:
        self.load2_regime = 0
      if self.load3_operating_current is None:
        self.load3_operating_current = 0
      if self.load3_regime is None:
        self.load3_regime = 0
      if self.load4_operating_current is None:
        self.load4_operating_current = 0
      if self.load4_regime is None:
        self.load4_regime = 0
      if self.load5_operating_current is None:
        self.load5_operating_current = 0
      if self.load5_regime is None:
        self.load5_regime = 0
      if self.bus28v1_voltage is None:
        self.bus28v1_voltage = 0
      if self.bus28v2_voltage is None:
        self.bus28v2_voltage = 0
      if self.bus28v3_voltage is None:
        self.bus28v3_voltage = 0
      if self.bus28v4_voltage is None:
        self.bus28v4_voltage = 0
      if self.bus270v1_voltage is None:
        self.bus270v1_voltage = 0
      if self.bus270v2_voltage is None:
        self.bus270v2_voltage = 0
      if self.bus270v3_voltage is None:
        self.bus270v3_voltage = 0
      if self.bus270v4_voltage is None:
        self.bus270v4_voltage = 0
      if self.aux_accumulator_regime is None:
        self.aux_accumulator_regime = 0
      if self.switch_state_left_power_module is None:
        self.switch_state_left_power_module = 0
      if self.switch_state_load is None:
        self.switch_state_load = 0
      if self.switch_state_right_power_module is None:
        self.switch_state_right_power_module = 0
      if self.reserve is None:
        self.reserve = 0
      if self.errors_cnt is None:
        self.errors_cnt = 0
      if self.errors_flag1 is None:
        self.errors_flag1 = 0
      if self.errors_flag2 is None:
        self.errors_flag2 = 0
    else:
      self.header = std_msgs.msg.Header()
      self.left_electromotor_current = 0
      self.left_electromotor_voltage = 0
      self.left_electromotor_temperature = 0
      self.left_electromotor_rate = 0
      self.left_electromotor_regime = 0
      self.right_electromotor_current = 0
      self.right_electromotor_voltage = 0
      self.right_electromotor_temperature = 0
      self.right_electromotor_rate = 0
      self.right_electromotor_regime = 0
      self.accumulator1_current = 0
      self.accumulator1_voltage = 0
      self.accumulator1_charge = 0
      self.accumulator1_temperature = 0
      self.accumulator1_regime = 0
      self.accumulator2_current = 0
      self.accumulator2_voltage = 0
      self.accumulator2_charge = 0
      self.accumulator2_temperature = 0
      self.accumulator2_regime = 0
      self.accumulator3_current = 0
      self.accumulator3_voltage = 0
      self.accumulator3_charge = 0
      self.accumulator3_temperature = 0
      self.accumulator3_regime = 0
      self.accumulator4_current = 0
      self.accumulator4_voltage = 0
      self.accumulator4_charge = 0
      self.accumulator4_temperature = 0
      self.accumulator4_regime = 0
      self.left_electromotor_start_current = 0
      self.right_electromotor_start_current = 0
      self.left_pump1_current = 0
      self.left_pump2_current = 0
      self.right_pump1_current = 0
      self.right_pump2_current = 0
      self.left_electromotor_operating_current = 0
      self.right_electromotor_operating_current = 0
      self.left_electromotor_servo_operating_current = 0
      self.left_engine_servo_operating_current = 0
      self.left_engine_servo_regime = 0
      self.right_engine_servo_operating_current = 0
      self.right_engine_servo_regime = 0
      self.load1_operating_current = 0
      self.load1_regime = 0
      self.load2_operating_current = 0
      self.load2_regime = 0
      self.load3_operating_current = 0
      self.load3_regime = 0
      self.load4_operating_current = 0
      self.load4_regime = 0
      self.load5_operating_current = 0
      self.load5_regime = 0
      self.bus28v1_voltage = 0
      self.bus28v2_voltage = 0
      self.bus28v3_voltage = 0
      self.bus28v4_voltage = 0
      self.bus270v1_voltage = 0
      self.bus270v2_voltage = 0
      self.bus270v3_voltage = 0
      self.bus270v4_voltage = 0
      self.aux_accumulator_regime = 0
      self.switch_state_left_power_module = 0
      self.switch_state_load = 0
      self.switch_state_right_power_module = 0
      self.reserve = 0
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
      buff.write(_struct_BHBH2BHBH2BH4BH4BH4BH30B4HB4H3B.pack(_x.left_electromotor_current, _x.left_electromotor_voltage, _x.left_electromotor_temperature, _x.left_electromotor_rate, _x.left_electromotor_regime, _x.right_electromotor_current, _x.right_electromotor_voltage, _x.right_electromotor_temperature, _x.right_electromotor_rate, _x.right_electromotor_regime, _x.accumulator1_current, _x.accumulator1_voltage, _x.accumulator1_charge, _x.accumulator1_temperature, _x.accumulator1_regime, _x.accumulator2_current, _x.accumulator2_voltage, _x.accumulator2_charge, _x.accumulator2_temperature, _x.accumulator2_regime, _x.accumulator3_current, _x.accumulator3_voltage, _x.accumulator3_charge, _x.accumulator3_temperature, _x.accumulator3_regime, _x.accumulator4_current, _x.accumulator4_voltage, _x.accumulator4_charge, _x.accumulator4_temperature, _x.accumulator4_regime, _x.left_electromotor_start_current, _x.right_electromotor_start_current, _x.left_pump1_current, _x.left_pump2_current, _x.right_pump1_current, _x.right_pump2_current, _x.left_electromotor_operating_current, _x.right_electromotor_operating_current, _x.left_electromotor_servo_operating_current, _x.left_engine_servo_operating_current, _x.left_engine_servo_regime, _x.right_engine_servo_operating_current, _x.right_engine_servo_regime, _x.load1_operating_current, _x.load1_regime, _x.load2_operating_current, _x.load2_regime, _x.load3_operating_current, _x.load3_regime, _x.load4_operating_current, _x.load4_regime, _x.load5_operating_current, _x.load5_regime, _x.bus28v1_voltage, _x.bus28v2_voltage, _x.bus28v3_voltage, _x.bus28v4_voltage, _x.bus270v1_voltage, _x.bus270v2_voltage, _x.bus270v3_voltage, _x.bus270v4_voltage, _x.aux_accumulator_regime, _x.switch_state_left_power_module, _x.switch_state_load, _x.switch_state_right_power_module, _x.reserve, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2))
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
      end += 85
      (_x.left_electromotor_current, _x.left_electromotor_voltage, _x.left_electromotor_temperature, _x.left_electromotor_rate, _x.left_electromotor_regime, _x.right_electromotor_current, _x.right_electromotor_voltage, _x.right_electromotor_temperature, _x.right_electromotor_rate, _x.right_electromotor_regime, _x.accumulator1_current, _x.accumulator1_voltage, _x.accumulator1_charge, _x.accumulator1_temperature, _x.accumulator1_regime, _x.accumulator2_current, _x.accumulator2_voltage, _x.accumulator2_charge, _x.accumulator2_temperature, _x.accumulator2_regime, _x.accumulator3_current, _x.accumulator3_voltage, _x.accumulator3_charge, _x.accumulator3_temperature, _x.accumulator3_regime, _x.accumulator4_current, _x.accumulator4_voltage, _x.accumulator4_charge, _x.accumulator4_temperature, _x.accumulator4_regime, _x.left_electromotor_start_current, _x.right_electromotor_start_current, _x.left_pump1_current, _x.left_pump2_current, _x.right_pump1_current, _x.right_pump2_current, _x.left_electromotor_operating_current, _x.right_electromotor_operating_current, _x.left_electromotor_servo_operating_current, _x.left_engine_servo_operating_current, _x.left_engine_servo_regime, _x.right_engine_servo_operating_current, _x.right_engine_servo_regime, _x.load1_operating_current, _x.load1_regime, _x.load2_operating_current, _x.load2_regime, _x.load3_operating_current, _x.load3_regime, _x.load4_operating_current, _x.load4_regime, _x.load5_operating_current, _x.load5_regime, _x.bus28v1_voltage, _x.bus28v2_voltage, _x.bus28v3_voltage, _x.bus28v4_voltage, _x.bus270v1_voltage, _x.bus270v2_voltage, _x.bus270v3_voltage, _x.bus270v4_voltage, _x.aux_accumulator_regime, _x.switch_state_left_power_module, _x.switch_state_load, _x.switch_state_right_power_module, _x.reserve, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2,) = _struct_BHBH2BHBH2BH4BH4BH4BH30B4HB4H3B.unpack(str[start:end])
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
      buff.write(_struct_BHBH2BHBH2BH4BH4BH4BH30B4HB4H3B.pack(_x.left_electromotor_current, _x.left_electromotor_voltage, _x.left_electromotor_temperature, _x.left_electromotor_rate, _x.left_electromotor_regime, _x.right_electromotor_current, _x.right_electromotor_voltage, _x.right_electromotor_temperature, _x.right_electromotor_rate, _x.right_electromotor_regime, _x.accumulator1_current, _x.accumulator1_voltage, _x.accumulator1_charge, _x.accumulator1_temperature, _x.accumulator1_regime, _x.accumulator2_current, _x.accumulator2_voltage, _x.accumulator2_charge, _x.accumulator2_temperature, _x.accumulator2_regime, _x.accumulator3_current, _x.accumulator3_voltage, _x.accumulator3_charge, _x.accumulator3_temperature, _x.accumulator3_regime, _x.accumulator4_current, _x.accumulator4_voltage, _x.accumulator4_charge, _x.accumulator4_temperature, _x.accumulator4_regime, _x.left_electromotor_start_current, _x.right_electromotor_start_current, _x.left_pump1_current, _x.left_pump2_current, _x.right_pump1_current, _x.right_pump2_current, _x.left_electromotor_operating_current, _x.right_electromotor_operating_current, _x.left_electromotor_servo_operating_current, _x.left_engine_servo_operating_current, _x.left_engine_servo_regime, _x.right_engine_servo_operating_current, _x.right_engine_servo_regime, _x.load1_operating_current, _x.load1_regime, _x.load2_operating_current, _x.load2_regime, _x.load3_operating_current, _x.load3_regime, _x.load4_operating_current, _x.load4_regime, _x.load5_operating_current, _x.load5_regime, _x.bus28v1_voltage, _x.bus28v2_voltage, _x.bus28v3_voltage, _x.bus28v4_voltage, _x.bus270v1_voltage, _x.bus270v2_voltage, _x.bus270v3_voltage, _x.bus270v4_voltage, _x.aux_accumulator_regime, _x.switch_state_left_power_module, _x.switch_state_load, _x.switch_state_right_power_module, _x.reserve, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2))
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
      end += 85
      (_x.left_electromotor_current, _x.left_electromotor_voltage, _x.left_electromotor_temperature, _x.left_electromotor_rate, _x.left_electromotor_regime, _x.right_electromotor_current, _x.right_electromotor_voltage, _x.right_electromotor_temperature, _x.right_electromotor_rate, _x.right_electromotor_regime, _x.accumulator1_current, _x.accumulator1_voltage, _x.accumulator1_charge, _x.accumulator1_temperature, _x.accumulator1_regime, _x.accumulator2_current, _x.accumulator2_voltage, _x.accumulator2_charge, _x.accumulator2_temperature, _x.accumulator2_regime, _x.accumulator3_current, _x.accumulator3_voltage, _x.accumulator3_charge, _x.accumulator3_temperature, _x.accumulator3_regime, _x.accumulator4_current, _x.accumulator4_voltage, _x.accumulator4_charge, _x.accumulator4_temperature, _x.accumulator4_regime, _x.left_electromotor_start_current, _x.right_electromotor_start_current, _x.left_pump1_current, _x.left_pump2_current, _x.right_pump1_current, _x.right_pump2_current, _x.left_electromotor_operating_current, _x.right_electromotor_operating_current, _x.left_electromotor_servo_operating_current, _x.left_engine_servo_operating_current, _x.left_engine_servo_regime, _x.right_engine_servo_operating_current, _x.right_engine_servo_regime, _x.load1_operating_current, _x.load1_regime, _x.load2_operating_current, _x.load2_regime, _x.load3_operating_current, _x.load3_regime, _x.load4_operating_current, _x.load4_regime, _x.load5_operating_current, _x.load5_regime, _x.bus28v1_voltage, _x.bus28v2_voltage, _x.bus28v3_voltage, _x.bus28v4_voltage, _x.bus270v1_voltage, _x.bus270v2_voltage, _x.bus270v3_voltage, _x.bus270v4_voltage, _x.aux_accumulator_regime, _x.switch_state_left_power_module, _x.switch_state_load, _x.switch_state_right_power_module, _x.reserve, _x.errors_cnt, _x.errors_flag1, _x.errors_flag2,) = _struct_BHBH2BHBH2BH4BH4BH4BH30B4HB4H3B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_BHBH2BHBH2BH4BH4BH4BH30B4HB4H3B = struct.Struct("<BHBH2BHBH2BH4BH4BH4BH30B4HB4H3B")