Header header

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