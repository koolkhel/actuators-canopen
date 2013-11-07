/*
 * canopen-data.h
 *
 *  Created on: 30.07.2013
 *      Author: yury
 */

#ifndef CANOPEN_DATA_H_
#define CANOPEN_DATA_H_

#include <canfestival.h>

#include "actuators-types.h"

// ensure everything is 1 byte aligned
#pragma pack(push,1)

union generator_205 {
	struct {
		UNS8 left_generator_start;
		UNS8 right_generator_start;
	};
	INTEGER64 data;
};
CTASSERT(sizeof(union generator_205) == 8);

union power_distribution_relay_305 {
	enum {
		AUTOMATIC = 0xAF,
		MANUAL = 0xBF
	};
	enum {
		ON = 0xA,
		OFF = 0x5
	};
	enum {
		BACKUP_AND_FAILSAFE_PARALLEL = 0x0A,
		BACKUP_MANUAL_AND_FAILSAFE_AUTOMATIC = 0x05
	};
	struct {
		// byte 0
		UNS8 control_type;

		// byte 1
		UNS8 right_power_equipment  :4;
		UNS8 left_power_equipment   :4;

		// byte 2
		UNS8 right_pressure_control :4;
		UNS8 left_pressure_control  :4;

		// byte 3
		UNS8 tail_270v_power_distribution: 4;
		UNS8 tail_28v_power_distribution:  4;

		// byte 4
		UNS8 top_signal_lights           :4;
		UNS8 backup_and_failsafe_battery :4;

		// byte 5
		UNS8 load_power           :4;
		UNS8 bottom_signal_lights :4;

		// byte 6
		UNS8 reserved_1;

		// byte 7
		UNS8 reserved_2;
	};
	INTEGER64 data;
};
CTASSERT(sizeof(union power_distribution_relay_305) == 8);

union left_main_engine_rotation_20A {
	struct {
		INTEGER16 left_main_engine_rotation_angle;
		UNS8 left_main_engine_rotation_fix;
	};
	INTEGER64 data;
};

union right_main_engine_rotation_20B {
	struct {
		INTEGER16 right_main_engine_rotation_angle;
		UNS8 right_main_engine_rotation_fix;
	};
	INTEGER64 data;
};

union left_main_engine_rotation_30A {
	struct {
		INTEGER16 left_main_engine_rotation_angle;
		UNS16 reserved;
	};
	INTEGER64 data;
};

union right_main_engine_rotation_30B {
	struct {
		INTEGER16 right_main_engine_rotation_angle;
		UNS16 reserved;
	};
	INTEGER64 data;
};

union tail_electromotor_207 {
#define ELECTROMOTOR_START 0xAA
#define ELECTROMOTOR_STOP 0x55
	struct {
		UNS8 left_electromotor_startstop;
		UNS8 right_electromotor_startstop;
	};
	INTEGER64 data;
};

union tail_electromotor_307 {
	struct {
		UNS16 left_electomotor_rate;
		UNS16 right_electromotor_rate;
	};
	INTEGER64 data;
};

union tail_electromotor_407 {
#define DO_FIX 0xAA
#define NO_FIX 0x55
	struct {
		INTEGER16 tail_engine_rotation_X_angle;
		INTEGER16 tail_engine_rotation_Y_angle;
		UNS8 tail_engine_rotation_X_fix;
		UNS8 tail_engine_rotation_Y_fix;
	};
	INTEGER64 data;
};

// посылаем 1 раз
// посылаем 3 раз
union left_main_engine_208_request {
	struct {
		UNS8 start_stop;
	};
	INTEGER64 data;
};

// приходит 2 раз
union left_main_engine_188_response {
	struct {
		UNS8 start_stop;
	};
	INTEGER64 data;
};

// посылаем 1 раз
// посылаем 3 раз
union right_main_engine_209_request {
	struct {
		UNS8 start_stop;
	};
	INTEGER64 data;
};

// приходит 2 раз
union left_main_engine_189_response {
	struct {
		UNS8 start_stop;
	};
	INTEGER64 data;
};

union left_main_engine_308 {
	struct {
		UNS8 regime; // 0xAA == rate, 0x55 == throttle
		UNS8 reserved;
		UNS16 rate;
		UNS16 throttle;
	};
	INTEGER64 data;
};

union right_main_engine_309 {
	struct {
		UNS8 regime; // 0xAA == rate, 0x55 == throttle
		UNS8 reserved;
		UNS16 rate;
		UNS16 throttle;
	};
	INTEGER64 data;
};

union left_main_engine_312 {
	struct main_engine_relay_command control;
	INTEGER64 data;
};

union right_main_engine_313 {
	struct main_engine_relay_command control;
	INTEGER64 data;
};

union left_ballonet_20c {
	struct {
		UNS16 left_ballonet_upper_pressure_threshold;
		UNS16 left_ballonet_upper_pressure_delta;
	};
	INTEGER64 data;
};

union right_ballonet_20d {
	struct {
		UNS16 right_ballonet_upper_pressure_threshold;
		UNS16 right_ballonet_upper_pressure_delta;
	};
	INTEGER64 data;
};

union left_ballonet_30c {
	struct {
		UNS16 left_ballonet_lower_pressure_threshold;
		UNS16 left_ballonet_lower_pressure_delta;
	};
	INTEGER64 data;
};

union right_ballonet_30d {
	struct {
		UNS16 right_ballonet_lower_pressure_threshold;
		UNS16 right_ballonet_lower_pressure_delta;
	};
	INTEGER64 data;
};

union left_ballonet_40c {
	struct {
		UNS8 left_ballonet_valve;

		UNS8 reserved_1;

		UNS8 left_ballonet_fan1;

		UNS8 reserved_2;

		UNS8 left_ballonet_fan2;

		UNS8 reserved_3;

		UNS8 left_ballonet_light;

		UNS8 reserved_4;
	};
	INTEGER64 data;
};

union right_ballonet_40d {
	struct {
		UNS8 right_ballonet_valve;

		UNS8 reserved_1;

		UNS8 right_ballonet_fan1;

		UNS8 reserved_2;

		UNS8 right_ballonet_fan2;

		UNS8 reserved_3;

		UNS8 right_ballonet_light;

		UNS8 reserved_4;
	};
	INTEGER64 data;
};

union left_ballonet_50c {
	struct {
		UNS16 regime;
	};
	INTEGER64 data;
};

union right_ballonet_50d {
	struct {
		UNS16 regime;
	};
	INTEGER64 data;
};

struct data_4002_20 {
	// p.5.2
	UNS16 failure_code;
};

struct data_4002_21 {
	UNS16 link_status;
};

struct data_4002_22 {
	UNS8 power_status[8];
};

struct data_4002_23 {
	UNS8 main_engine_status[4];
};

struct data_4002_24 {
	UNS8 rotation_status[10];
};

struct data_4002_25 {
	UNS8 misc_status[8];
};

struct data_4003_20 {
	UNS16 gps_validity; // 00 == OK, 01 == BAD

	// 0.0001 minute
	UNS32 latitude;

	// 0.0001 minute
	UNS32 longitude;

	// 1m
	INTEGER16 height_sea_level;

	// 0.01 m/s
	INTEGER16 speed_east_west;

	// 0.01 m/s
	INTEGER16 speed_north_south;

	// 0.01 m/s
	INTEGER16 speed_top_down;

	// cren 0.01 degree
	INTEGER16 roll_angle;

	// tangazh 0.01 degree
	INTEGER16 pitch_angle;

	// ryskanie 0.01 degree
	INTEGER16 yaw_angle;

	// 0.01 %
	INTEGER16 roll_rate;

	// 0.01%
	INTEGER16 pitch_rate;

	// 0.01%
	INTEGER16 yaw_rate;

	// 1 mg
	INTEGER16 acceleration_X;

	// 1 mg
	INTEGER16 acceleration_Y;

	// 1 mg
	INTEGER16 acceleration_Z;

	// 0.01 m/s
	INTEGER16 air_speed;

	// 1m
	INTEGER16 altitude_by_pressure;

	// 1m
	INTEGER16 altitude_by_land;

	// 0.1 degree
	INTEGER16 load_temperature;

	// 1% RH
	UNS16 load_humidity;

	// 0.1 C
	INTEGER16 environment_temperature;

	// 1%RH
	UNS16 environment_humidity;

	// 0.01 m/s
	INTEGER16 air_speed_X;

	// 0.01 m/s
	INTEGER16 air_speed_Y;

	// 0.01 m/s
	INTEGER16 air_speed_Z;
};

struct data_4004_20 {};

struct PACKED data_4005_21 {
	// 0.1A
	UNS16 _01_left_main_engine_generator_output_current;

	// 0.1V
	UNS16 _02_left_main_engine_generator_output_voltage;

	// 0.1A
	UNS16 _03_right_main_engine_generator_output_current;

	// 0.1V
	UNS16 _04_right_main_engine_generator_output_voltage;

	// 0.1A
	UNS16 _05_left_ballonet_control_current;

	// 0.1A
	UNS16 _06_right_ballonet_control_current;

	// 0,1A
	UNS16 _07_power_section_28v_power_equipment_current;

	// 0,1A
	UNS16 _08_tail_section_28v_power_equipment_current;

	// 0.1V
	UNS16 _09_bus_backup_28v_voltage;

	// 0,1V
	UNS16 _10_failsafe_28v_voltage;

	// 0.1V
	UNS16 _11_backup_battery_voltage;

	// 0,1V
	UNS16 _12_failsafe_battery_voltage;

	// 0.1A
	UNS16 _13_backup_battery_charge_current;

	// 0.1A
	UNS16 _14_backup_battery_discharge_current;

	// 1C
	UNS16 _15_backup_battery_monoblock_temperature;

	// примечание 1
	struct backup_battery_state _16_backup_battery_failure_state;

	// 0.1A
	UNS16 _17_failsafe_battery_charge_current;

	// 0.1A
	UNS16 _18_failsafe_battery_discharge_current;

	// 1C
	UNS16 _19_failsafe_battery_monoblock_temperature;

	// примечание 2
	struct failsafe_battery_state _20_failsafe_battery_failure_state;

	// 0.1A
	UNS16 _21_left_charge_device_output_current;

	// reserved
	UNS16 _22_left_charge_device_state;

	// 0.1A
	UNS16 _23_right_charge_device_output_current;

	// reserved
	UNS16 _24_right_charge_device_state;

	// примечание 3
	struct power_distribution_relay_state _25_distribution_relay_state;

	// 0.1A
	UNS16 _26_load_equipment_28v_current;

	// примечание 4
	struct power_distribution_line_failure _27_distribution_lines_failure_state;
};
CTASSERT(sizeof(UNS16) == 2);
CTASSERT(sizeof(struct backup_battery_state) == 2);
CTASSERT(sizeof(struct main_engine_relay_state) == 2);
CTASSERT(sizeof(struct main_engine_relay_command) == 8);
CTASSERT(sizeof(struct power_distribution_relay_state) == 2);
CTASSERT(sizeof(struct power_distribution_line_failure) == 2);
CTASSERT(sizeof(struct failsafe_battery_state) == 2);
CTASSERT(sizeof(struct data_4005_21) == 2 * 27);

struct data_4007_20 {
	// * 1
	UNS16 left_electromotor_rate;
	// * 1
	UNS16 right_electromotor_rate;

	// / 100
	INTEGER16 left_electromotor_angle_X;

	// / 100
	INTEGER16 right_electromotor_angle_X;


	// / 100
	INTEGER16 left_electromotor_angle_Y;

	// / 100
	INTEGER16 right_electromotor_angle_Y;

	INTEGER32 reserved;
};

struct data_4007_21 {
	// / 10 V
	UNS16 left_electromotor_voltage;

	// / 10 V
	UNS16 right_electromotor_voltage;

	// / 10 A
	UNS16 left_electromotor_current;

	// / 10 A
	UNS16 right_electromotor_current;

	// / 10 C
	INTEGER16 left_electromotor_temperature;

	// / 10 C
	INTEGER16 right_electromotor_temperature;

	// / 10 C
	INTEGER16 left_electromotor_rotation_temperature;

	// / 10 C
	INTEGER16 right_electromotor_rotation_temperature;

	INTEGER32 reserved;
};

struct data_4008_20 {
	// 1 rpm
	UNS16 left_main_engine_rate;

	// 0.1%
	UNS16 left_main_engine_fuel_level;

	// 0.1%
	UNS16 left_main_engine_aux_fuel_level;

	// 0.1 C
	UNS16 left_main_engine_exhaust_temperature;

	// 0.1 C
	UNS16 left_main_engine_cylinder_1_temperature;

	// 0.1 C
	UNS16 left_main_engine_cylinder_2_temperature;

	// 0.1 degree
	UNS16 left_main_engine_throttle_1_angle;

	// 0.1 degree
	UNS16 left_main_engine_throttle_2_angle;

	// состояние реле
	struct main_engine_relay_state left_main_engine_relay_state;

	UNS16 reserved;
};

struct data_4009_20 {
	// 1 rpm
	UNS16 right_main_engine_rate;

	// 0.1%
	UNS16 right_main_engine_fuel_level;

	// 0.1%
	UNS16 right_main_engine_aux_fuel_level;

	// 0.1 C
	UNS16 right_main_engine_exhaust_temperature;

	// 0.1 C
	UNS16 right_main_engine_cylinder_1_temperature;

	// 0.1 C
	UNS16 right_main_engine_cylinder_2_temperature;

	// 0.1 degree
	UNS16 right_main_engine_throttle_1_angle;

	// 0.1 degree
	UNS16 right_main_engine_throttle_2_angle;

	struct main_engine_relay_state right_main_engine_relay_state;

	UNS16 reserved;
};

struct data_400a_20 {
	// 0.01 degree
	INTEGER16 left_main_engine_rotation_angle;

	UNS16 reserved;
};

struct data_400a_21 {
	// 0.1A
	UNS16 left_main_engine_rotation_current;
	// 0.1V
	UNS16 left_main_engine_rotation_voltage;
};

struct data_400b_20 {
	// 0.01 degree
	INTEGER16 right_main_engine_rotation_angle;

	UNS16 reserved;
};

struct data_400b_21 {
	// 0.1A
	UNS16 right_main_engine_rotation_current;
	// 0.1V
	UNS16 right_main_engine_rotation_voltage;
};

struct data_400c_20 {
	// 1Pa
	UNS16 left_ballonet_pressure_1;

	// 1 Pa
	UNS16 left_ballonet_pressure_2;

	UNS8 left_ballonet_lights_state;

	UNS8 reserved;

	UNS16 left_ballonet_control;

	UNS16 left_ballonet_linear_valve_resistance;
};

struct data_400d_20 {
	// 1Pa
	UNS16 right_ballonet_pressure_1;

	// 1 Pa
	UNS16 right_ballonet_pressure_2;

	UNS8 right_ballonet_lights_state;

	UNS8 reserved;

	UNS16 right_ballonet_control;

	UNS16 right_ballonet_linear_valve_resistance;
};

#pragma pack(pop)

#endif /* CANOPEN_DATA_H_ */
