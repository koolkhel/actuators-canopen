Header header

byte FAILURE_FLAG_OK = 0
byte FAILURE_FLAG_FAILURE = 1

byte FAILURE_CODE_OK = 0
byte FAILURE_CODE_CHARGE_ONLY = 1
byte FAILURE_CODE_DISCHARGE_ONLY = 2
byte FAILURE_CODE_FAILURE = 3

uint8 failsafe_battery1_failure_flag
uint8 failsafe_battery1_failure_code

uint8 failsafe_battery2_failure_flag
uint8 failsafe_battery2_failure_code
