/*
 * model.h
 *
 *  Created on: 03.12.2012
 *      Author: yury
 */
#pragma once

#include <pthread.h>
#include <canfestival.h>

#include "actuators-types.h"
#include "actuators.h"

enum PACKED engine_fix {
        MAIN_ENGINE_FIX_FORCE = 0xAA,
        MAIN_ENGINE_FIX_NONE  = 0x55,
};

#pragma pack(push, 1)

#define RESET_COLOR "\e[m"
#define MAKE_GREEN "\e[32m"
#define MAKE_BLUE "\e[34m"
#define MAKE_YELLOW "\e[33m"

/**
 * all domain level variables are grouped in one struct
 */
struct actuators_model {
	UNS8 remote_control_allowed_state;

	volatile UNS8 remote_control_enabled;
#if 0
	// struct data_4002_20
	UNS16 failure_code;

	// struct data_4002_21
	UNS16 link_status;


	// struct data_4002_22
	UNS8 power_status[8];


	//struct data_4002_23
	UNS8 main_engine_status[4];

	// struct data_4002_24
	UNS8 rotation_status[10];


	//struct data_4002_25
	UNS8 misc_status[8];


	// struct data_4003_20
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
#endif

	//struct data_4005_21
	// 0.1A
	float left_main_engine_generator_output_current;

	// 0.1V
	float left_main_engine_generator_output_voltage;

	// 0.1A
	float right_main_engine_generator_output_current;

	// 0.1V
	float right_main_engine_generator_output_voltage;

	// 0.1A
	float left_ballonet_control_current;

	// 0.1A
	float right_ballonet_control_current;

	// 0,1A
	float power_section_28v_power_equipment_current;

	// 0,1A
	float tail_section_28v_power_equipment_current;

	// 0.1V
	float bus_backup_28v_voltage;

	// 0,1V
	float failsafe_28v_voltage;

	// 0.1V
	float backup_battery_voltage;

	// 0,1V
	float failsafe_battery_voltage;

	// 0.1A
	float backup_battery_charge_current;

	// 0.1A
	float backup_battery_discharge_current;

	// 1C
	float backup_battery_monoblock_temperature;

	// примечание 1
	struct backup_battery_state backup_battery_failure_state;

	// 0.1A
	float failsafe_battery_charge_current;

	// 0.1A
	float failsafe_battery_discharge_current;

	// 1C
	float failsafe_battery_monoblock_temperature;

	// примечание 2
	struct failsafe_battery_state failsafe_battery_failure_state;

	// 0.1A
	float left_charge_device_output_current;

	// reserved
	float left_charge_device_state;

	// 0.1A
	float right_charge_device_output_current;

	// reserved
	float right_charge_device_state;

	// примечание 3
	struct power_distribution_relay_state distribution_relay_state;

	// 0.1A
	float load_equipment_28v_current;

	// примечание 4
	struct power_distribution_line_failure distribution_lines_failure_state;

	// struct data_4006_20
	// 0.01 degree
	float left_control_surface_X;

	// 0.01 degree
	float left_control_surface_Y;

	// 0.01 degree
	float right_control_surface_X;

	// 0.01 degree
	float right_control_surface_Y;

	// struct data_4006_21
	// 0.1 A
	float control_surface_1_current;

	// 0.1 A
	float control_surface_2_current;

	// 0.1 A
	float control_surface_3_current;

	// 0.1 A
	float control_surface_4_current;

	// 0.1 A
	float control_surface_5_current;

	// 0.1 A
	float control_surface_6_current;

	// 0.1 A
	float control_surface_7_current;

	// 0.1 A
	float control_surface_8_current;

	// struct data_4007_20
	// * 1
	UNS16 left_electromotor_rate;
	// * 1
	UNS16 right_electromotor_rate;

	// / 100
	float left_electromotor_angle_Y;

	// / 100
	float right_electromotor_angle_Y;

	// / 100
	float left_electromotor_angle_X;

	// / 100
	float right_electromotor_angle_X;

	INTEGER32 reserved_2;


	//struct data_4007_21 {
	// / 10 V
	float left_electromotor_voltage;

	// / 10 V
	float right_electromotor_voltage;

	// / 10 A
	float left_electromotor_current;

	// / 10 A
	float right_electromotor_current;

	// / 10 C
	float left_electromotor_temperature;

	// / 10 C
	float right_electromotor_temperature;

	// / 10 C
	float left_electromotor_rotation_temperature;

	// / 10 C
	float right_electromotor_rotation_temperature;

	INTEGER32 reserved_3;


	// struct data_4008_20
	// 1 rpm
	UNS16 left_main_engine_rate;

	// 0.1%
	float left_main_engine_fuel_level;

	// 0.1%
	float left_main_engine_aux_fuel_level;

	// 0.1 C
	float left_main_engine_exhaust_temperature;

	// 0.1 C
	float left_main_engine_cylinder_1_temperature;

	// 0.1 C
	float left_main_engine_cylinder_2_temperature;

	// 0.1 degree
	float left_main_engine_throttle_1_angle;

	// 0.1 degree
	float left_main_engine_throttle_2_angle;

	struct main_engine_relay_state left_main_engine_relay_state;

	INTEGER16 reserved_4;


	//struct data_4009_20
	// 1 rpm
	UNS16 right_main_engine_rate;

	// 0.1%
	float right_main_engine_fuel_level;

	// 0.1%
	float right_main_engine_aux_fuel_level;

	// 0.1 C
	float right_main_engine_exhaust_temperature;

	// 0.1 C
	float right_main_engine_cylinder_1_temperature;

	// 0.1 C
	float right_main_engine_cylinder_2_temperature;

	// 0.1 degree
	float right_main_engine_throttle_1_angle;

	// 0.1 degree
	float right_main_engine_throttle_2_angle;

	struct main_engine_relay_state right_main_engine_relay_state;

	INTEGER16 reserved_5;


	//struct data_400a_20
	// 0.01 degree
	float left_main_engine_rotation_angle;




	//struct data_400a_21
	// 0.1V
	float left_main_engine_rotation_voltage;

	float left_main_engine_rotation_current;


	// struct data_400b_20
	// 0.01 degree
	float right_main_engine_rotation_angle;

	// struct data_400b_21
	float right_main_engine_rotation_current;

	float right_main_engine_rotation_voltage;


	//struct data_400c_20
	// 1Pa
	UNS16 left_ballonet_pressure_1;

	// 1 Pa
	UNS16 left_ballonet_pressure_2;

	UNS16 left_ballonet_lights_state;

	UNS16 left_ballonet_control;

	float left_ballonet_linear_valve_resistance;


	// struct data_400d_20
	// 1Pa
	UNS16 right_ballonet_pressure_1;

	//
	UNS16 right_ballonet_lights_state;

	// 1 Pa
	UNS16 right_ballonet_pressure_2;

	UNS16 right_ballonet_control;

	float right_ballonet_linear_valve_resistance;

	// struct data_400e_20
	UNS16 helium_valve_open;
};

extern pthread_mutex_t REPORTED_DATA_lock;
extern pthread_mutex_t MODEL_lock;
extern struct actuators_model REPORTED_DATA;
extern struct actuators_model MODEL;

#define LOCK_MODEL() do {\
	int result = pthread_mutex_lock(&MODEL_lock);\
	if (result) {char buf[200]; fprintf(stderr, "mutex lock error: %s\n", strerror_r(result, buf, 200));}; \
} while (0)

#define UNLOCK_MODEL() do {\
	int result = pthread_mutex_unlock(&MODEL_lock); \
	if (result) {char buf[200]; fprintf(stderr, "mutex unlock error: %s\n", strerror_r(result, buf, 200));}; \
} while (0)

#define LOCK_REPORTED() do {\
	pthread_mutex_lock(&REPORTED_DATA_lock);\
} while (0)

#define UNLOCK_REPORTED() do {\
	pthread_mutex_unlock(&REPORTED_DATA_lock); \
} while (0)

#pragma pack(pop)
