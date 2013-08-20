/*
 * canopen-data.h
 *
 *  Created on: 30.07.2013
 *      Author: yury
 */

#ifndef CANOPEN_DATA_H_
#define CANOPEN_DATA_H_

#include <canfestival.h>

// ensure everything is 1 byte aligned
#pragma pack(push,1)

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

struct data_4005_21 {
	// 1A
	UNS8 left_electromotor_output_current;

	// 1V
	UNS16 left_electromotor_output_voltage;

	// 1A
	UNS8 right_electromotor_output_current;

	// 1V
	UNS16 right_electromotor_output_voltage;

	UNS8 electromotor_relay_status;

	// 0.1 V
	UNS16 power_failsafe_28V_voltage;

	// 0.1V
	UNS16 power_aux_28V_voltage;

	// 1A
	UNS8 left_ballonet_current;

	// 1A
	UNS8 right_ballonet_current;

	// 1A
	UNS8 power_tail_supply_current_28V;

	// 1A "отсев"
	UNS8 power_dropout_supply_current_28V;

	// 1A
	UNS8 power_board_supply_current_28V;

	// ?
	UNS16 power_distributor;

	// 0.1V
	UNS16 power_backup_voltage;

	// 1A
	UNS8 power_backup_current;

	// 1A "разрядный ток"
	UNS8 power_backup_current_drain;

	// %?
	UNS8 power_backup_capacity;

	UNS16 power_backup_status;

	// 0.1V
	UNS16 power_failsafe_voltage;

	// 1A
	UNS8 power_failsafe_charge_current;

	// 1A
	UNS8 power_failsafe_drain_current;

	UNS8 power_failsafe_capacity;

	UNS16 power_failsafe_status;

	// 1A
	UNS8 power_charger_1_output_current;

	UNS8 power_charger_1_status;

	UNS8 power_charger_2_output_current;

	UNS8 power_charger_2_status;

	UNS32 reserved;
};

struct data_4007_20 {
	// * 1
	UNS16 left_electromotor_rate;
	// * 1
	UNS16 right_electromotor_rate;

	// / 100
	INTEGER16 left_electromotor_angle_Y;

	// / 100
	INTEGER16 right_electromotor_angle_Y;

	// / 100
	INTEGER16 left_electromotor_angle_X;

	// / 100
	INTEGER16 right_electromotor_angle_X;

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
	UNS16 left_electromotor_rate;

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
	UNS16 left_main_engine_throttle_1;

	// 0.1 degree
	UNS16 left_main_engine_throttle_2;

	INTEGER32 reserved;
};

struct data_4009_20 {
	// 1 rpm
	UNS16 right_electromotor_rate;

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
	UNS16 right_main_engine_throttle_1;

	// 0.1 degree
	UNS16 right_main_engine_throttle_2;

	INTEGER32 reserved;
};

struct data_400a_20 {
	// 0.01 degree
	UNS16 left_main_engine_rotation_angle;

	UNS16 reserved;
};

struct data_400a_21 {
	// 0.1V
	UNS16 left_main_engine_rotation_voltage;

	UNS16 reserved;
};

struct data_400b_20 {
	// 0.01 degree
	UNS16 right_main_engine_rotation_angle;

	UNS16 reserved;
};

struct data_400c_20 {
	// 1Pa
	UNS16 left_ballonet_pressure_1;

	// 1 Pa
	UNS16 left_ballonet_pressure_2;

	UNS16 left_ballonet_control;

	UNS16 reserved;
};

struct data_400d_20 {
	// 1Pa
	UNS16 right_ballonet_pressure_1;

	// 1 Pa
	UNS16 right_ballonet_pressure_2;

	UNS16 right_ballonet_control;

	UNS16 reserved;
};

#pragma pack(pop)

#endif /* CANOPEN_DATA_H_ */
