#pragma once

#include <stdint.h>

typedef unsigned char BYTE;
typedef unsigned short int WORD;
typedef unsigned int DWORD;
typedef short int SWORD;
typedef float FLOAT;

typedef unsigned char BYTE;
// table B

#pragma pack(push,1)

#ifndef CTASSERT /* Allow lint to override */
#define CTASSERT(x) _CTASSERT(x, __LINE__)
#define _CTASSERT(x, y) __CTASSERT(x, y)
#define __CTASSERT(x, y) typedef char __assert ## y[(x) ? 1 : -1]
#endif

struct main_engine_relay_state {
	BYTE ignition_1:1;
	BYTE ignition_2:1;
	BYTE servo:1;
	BYTE magnetic_valve:1; // reserved
	BYTE ECU:1;
	BYTE electromotor_start:1;
	BYTE aux_fuel_pump:1; // reserved
	BYTE main_fuel_pump:1;

	BYTE reserved;
};

// 0xAA == on, 0x55 == off
struct main_engine_relay_command {
	UNS8 ignition_1;
	UNS8 ignition_2;
	UNS8 servo;
	UNS8 magnetic_valve; // reserved
	UNS8 ECU;
	UNS8 electromotor_start;
	UNS8 aux_fuel_pump; // reserved
	UNS8 main_fuel_pump;
};

// POWER MANAGEMENT

enum battery_failure_flag_t {
	BATTERY_FLAG_OK = 0,
	BATTERY_FLAG_FAILURE = 1
};

enum battery_failure_code_t {
	BATTERY_CODE_OK = 0,
	BATTERY_CODE_CHARGE_ONLY = 1,
	BATTERY_CODE_DISCHARGE_ONLY = 2,
	BATTERY_CODE_FAILURE = 3 // cannot charge and cannot discharge
};

struct backup_battery_state {
	// battery1
	UNS8 battery1_failure_flag:1;
	UNS8 battery1_failure_code:2;

	UNS8 battery2_failure_flag:1;
	UNS8 battery2_failure_code:2;

	UNS8 battery3_failure_flag:1;
	UNS8 battery3_failure_code:2;
};

struct failsafe_battery_state {
	UNS8 failsafe_battery1_failure_flag:1;
	UNS8 failsafe_battery1_failure_code:2;

	UNS8 failsafe_battery2_failure_flag:1;
	UNS8 failsafe_battery2_failure_code:2;

	UNS8 reserved;
};

struct power_distribution_relay_state {
	UNS8 both_battery_power     :1;
	UNS8 reserved               :1;
	UNS8 bottom_signal_lights   :1;
	UNS8 top_signal_lights      :1;
	UNS8 load                   :1;
	UNS8 tail_28v_equipment     :1;
	UNS8 right_pressure_control :1;
	UNS8 left_pressure_control  :1;
	UNS8 right_power_equipment  :1;
	UNS8 left_power_equipment   :1;
};

// 0 == ok, 1 == failure
struct power_distribution_line_failure {
	UNS8 bottom_signal_lights_failure  :1;
	UNS8 top_signal_lights_failure     :1;
	UNS8 tail_equipment_failure        :1;
	UNS8 left_power_equipment_failure  :1;
	UNS8 right_power_equipment_failure :1;

	UNS8 reserved;
};

#define PACKED __attribute__ ((__packed__))

/// Byte swap unsigned short
#define FLIP_ENDIAN_WORD(val) ((((val) << 8) & 0xFF00) | (((val) >> 8) & 0xFF))

#ifdef DEBUG
#define TRACE_ENTRY() fprintf(stderr, "entering %s\n", __func__)
#define TRACE_EXIT() fprintf(stderr, "exiting %s\n", __func__)
#else
#define TRACE_ENTRY() do {} while (0)
#define TRACE_EXIT() do {} while (0)
#endif

#pragma pack(pop)
