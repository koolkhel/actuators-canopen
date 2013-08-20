Header header

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