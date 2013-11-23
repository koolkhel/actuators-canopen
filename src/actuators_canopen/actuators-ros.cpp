/*
 * actuators-ros.cpp
 *
 *  Created on: 15.12.2012
 *      Author: yury
 */
#include <ros/ros.h>
#include <ros/console.h>

#include <signal.h>

#include "actuators-ros.h"

#include "actuators/BallonetState.h"
#include "actuators/BallonetStateExt.h"

#include "actuators/ElectromotorState.h"
#include "actuators/ElectromotorsState.h"
#include "actuators/ElectromotorServoState.h"
#include "actuators/ElectromotorsStateExt.h"

#include "actuators/MotorsControl.h"

#include "actuators/EngineState.h"
#include "actuators/EngineStateExt.h"
#include "actuators/EngineServoState.h"
#include "actuators/EngineServoStateExt.h"

#include "actuators/StartEngine.h"
#include "actuators/StopEngine.h"

#include "actuators/SetEngineRelayState.h"
#include "actuators/SetMotorsControl.h"
#include "actuators/SetBallonetControl.h"
#include "actuators/SetElectromotorsOnOff.h"
#include "actuators/SwitchHeliumValve.h"
#include "actuators/BallonetRegime.h"
#include "actuators/SetBallonetRegime.h"
#include "actuators/SetBallonetAutomaticControlParameters.h"
#include "actuators/SetGeneratorControl.h"
#include "actuators/SetPowerRelay.h"

#include "actuators/SetControlSurface.h"
#include "actuators/ControlSurfaceState.h"

#include "actuators/SetRemoteControlAllow.h"
#include "actuators/RemoteControlState.h"

#include "actuators/BatteryBackupState.h"
#include "actuators/BatteryFailsafeState.h"
#include "actuators/PowerDistributionLineFailure.h"
#include "actuators/PowerDistributionRelayState.h"

#include "actuators/PowerSystemState.h"

#include "actuators/HeliumValveState.h"

#include "model.h"
#include "canopen-data.h"
#include "canopen-util.h"

#include "matlab-connector.h"

static ros::ServiceServer start_left_engine_service;
static ros::ServiceServer stop_left_engine_service;
static ros::ServiceServer set_left_engine_relay_state_service;

static ros::ServiceServer set_power_relay_service;
static ros::ServiceServer set_generator_control_service;

static ros::ServiceServer start_right_engine_service;
static ros::ServiceServer stop_right_engine_service;
static ros::ServiceServer set_right_engine_relay_state_service;

static ros::ServiceServer set_motors_control_service;
static ros::ServiceServer set_tail_electromotor_onoff_service;

static ros::ServiceServer set_left_ballonet_regime_service;
static ros::ServiceServer set_left_ballonet_automatic_control_parameters_service;
static ros::ServiceServer set_left_ballonet_control_service;

static ros::ServiceServer set_right_ballonet_regime_service;
static ros::ServiceServer set_right_ballonet_automatic_control_parameters_service;
static ros::ServiceServer set_right_ballonet_control_service;

static ros::ServiceServer switch_helium_valve_service;

static ros::ServiceServer set_remote_control_allow_service;

static ros::ServiceServer set_control_surface_service;

bool set_remote_control_allow(actuators::SetRemoteControlAllowRequest &req, actuators::SetRemoteControlAllowResponse &resp) {
	UNS32 size = 0;
	int result = 0;

	ROS_ERROR("set_remote_control_allow %hhu", req.remote_control_allow);

	union remote_control_allow_201 command;
	command.data = 0;
	switch (req.remote_control_allow) {
	case 0:
		command.remote_control_allow = 0x55;
		MODEL.remote_control_allowed_state = 0;
		break;
	case 1:
		command.remote_control_allow = 0xAA;
		MODEL.remote_control_allowed_state = 1;
		break;
	}

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5019, 0x0, &command.data, &size, 0);

	return true;
}

static void set_left_ballonet_upper_pressure(UNS32 left_ballonet_upper_pressure_threshold,
		UNS32 left_ballonet_upper_pressure_delta) {
	UNS32 size = 0;
	int result = 0;

	union left_ballonet_20c command;
	command.data = 0;
	command.left_ballonet_upper_pressure_threshold = (UNS16) left_ballonet_upper_pressure_threshold;
	command.left_ballonet_upper_pressure_delta = (UNS16) left_ballonet_upper_pressure_delta;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5004, 0x0, &command.data, &size, 0);

}

static void set_left_ballonet_lower_pressure(UNS32 left_ballonet_lower_pressure_threshold,
		UNS32 left_ballonet_lower_pressure_delta) {
	UNS32 size = 0;
	int result = 0;

	union left_ballonet_30c command;
	command.data = 0;
	command.left_ballonet_lower_pressure_threshold = (UNS16) left_ballonet_lower_pressure_threshold;
	command.left_ballonet_lower_pressure_delta = (UNS16) left_ballonet_lower_pressure_delta;


	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5005, 0x0, &command.data, &size, 0);

}

static void _set_left_ballonet_control(UNS8 fan1, UNS8 fan2, UNS8 valve, UNS8 light) {
	UNS32 size = 0;
	int result = 0;

	union left_ballonet_40c command;
	command.data = 0;
	command.left_ballonet_fan1 = fan1 ? 0xAA : 0x55;
	command.left_ballonet_fan2 = fan2 ? 0xAA : 0x55;
	command.left_ballonet_valve = valve ? 0xAA : 0x55;
	command.left_ballonet_light = light ? 0xAA : 0x55;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5006, 0x0, &command.data, &size, 0);
}

static bool set_left_ballonet_control(actuators::SetBallonetControlRequest &req, actuators::SetBallonetControlResponse &resp) {

	ROS_ERROR("set_left_ballonet_control: FAN1 %hhu FAN2 %hhu VALVE %hhu LIGHT %hhu", req.fan1, req.fan2, req.valve, req.light);

	_set_left_ballonet_control(req.fan1, req.fan2, req.valve, req.light);

	enqueue_PDO(0x6);
	return true;
}

static bool set_left_ballonet_regime(actuators::SetBallonetRegimeRequest &req,
		actuators::SetBallonetRegimeResponse &resp) {
	UNS32 size = 0;
	int result = 0;

	union left_ballonet_50c command;
	command.data = 0;

	ROS_ERROR("set_left_ballonet_regime: %s", req.regime.regime ? "automatic" : "manual");

	switch (req.regime.regime) {
	case 0:
		command.regime = 0x51;
		break;
	case 1:
		command.regime = 0x61;
		break;
	}

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5007, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x7);

	return true;
}

static bool set_left_ballonet_automatic_control_parameters(
		actuators::SetBallonetAutomaticControlParametersRequest &req,
		actuators::SetBallonetAutomaticControlParametersResponse &response) {

	ROS_ERROR("set_left_ballonet_automatic_control_parameters: %d %d %d %d",
			req.ballonet_upper_pressure_threshold,
			req.ballonet_upper_pressure_delta,
			req.ballonet_lower_pressure_threshold,
			req.ballonet_lower_pressure_delta);

	set_left_ballonet_upper_pressure(req.ballonet_upper_pressure_threshold,
			req.ballonet_upper_pressure_delta);

	enqueue_PDO(0x4);

	set_left_ballonet_lower_pressure(req.ballonet_lower_pressure_threshold,
				req.ballonet_lower_pressure_delta);

	enqueue_PDO(0x5);

	return true;
}

static void set_right_ballonet_upper_pressure(UNS32 right_ballonet_upper_pressure_threshold,
		UNS32 right_ballonet_upper_pressure_delta) {
	UNS32 size = 0;
	int result = 0;

	union right_ballonet_20d command;
	command.data = 0;
	command.right_ballonet_upper_pressure_threshold = (UNS16) right_ballonet_upper_pressure_threshold;
	command.right_ballonet_upper_pressure_delta = (UNS16) right_ballonet_upper_pressure_delta;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5008, 0x0, &command.data, &size, 0);

}

static void set_right_ballonet_lower_pressure(UNS32 right_ballonet_lower_pressure_threshold,
		UNS32 right_ballonet_lower_pressure_delta) {
	UNS32 size = 0;
	int result = 0;

	union right_ballonet_30d command;
	command.data = 0;
	command.right_ballonet_lower_pressure_threshold = (UNS16) right_ballonet_lower_pressure_threshold;
	command.right_ballonet_lower_pressure_delta = (UNS16) right_ballonet_lower_pressure_delta;


	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5009, 0x0, &command.data, &size, 0);

}

static void _set_right_ballonet_control(UNS8 fan1, UNS8 fan2, UNS8 valve, UNS8 light) {
	UNS32 size = 0;
	int result = 0;

	union right_ballonet_40d command;
	command.data = 0;
	command.right_ballonet_fan1 = fan1 ? 0xAA : 0x55;
	command.right_ballonet_fan2 = fan2 ? 0xAA : 0x55;
	command.right_ballonet_valve = valve ? 0xAA : 0x55;
	command.right_ballonet_light = light ? 0xAA : 0x55;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x500a, 0x0, &command.data, &size, 0);
}

static bool set_right_ballonet_control(actuators::SetBallonetControlRequest &req, actuators::SetBallonetControlResponse &resp) {

	ROS_ERROR("set_right_ballonet_control: FAN1 %hhu FAN2 %hhu VALVE %hhu LIGHT %hhu", req.fan1, req.fan2, req.valve, req.light);

	_set_right_ballonet_control(req.fan1, req.fan2, req.valve, req.light);

	enqueue_PDO(0x0A);
	return true;
}

static bool set_right_ballonet_regime(actuators::SetBallonetRegimeRequest &req,
		actuators::SetBallonetRegimeResponse &resp) {
	UNS32 size = 0;
	int result = 0;

	union right_ballonet_50d command;
	command.data = 0;

	ROS_ERROR("set_right_ballonet_regime: %s", req.regime.regime ? "automatic" : "manual");

	switch (req.regime.regime) {
	case 0:
		command.regime = 0x51;
		break;
	case 1:
		command.regime = 0x61;
		break;
	}

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x500b, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x0B);

	return true;
}

static bool set_right_ballonet_automatic_control_parameters(
		actuators::SetBallonetAutomaticControlParametersRequest &req,
		actuators::SetBallonetAutomaticControlParametersResponse &response) {

	ROS_ERROR("set_right_ballonet_automatic_control_parameters: %d %d %d %d",
			req.ballonet_upper_pressure_threshold,
			req.ballonet_upper_pressure_delta,
			req.ballonet_lower_pressure_threshold,
			req.ballonet_lower_pressure_delta);

	set_right_ballonet_upper_pressure(req.ballonet_upper_pressure_threshold,
			req.ballonet_upper_pressure_delta);

	enqueue_PDO(0x08);

	set_right_ballonet_lower_pressure(req.ballonet_lower_pressure_threshold,
				req.ballonet_lower_pressure_delta);

	enqueue_PDO(0x09);

	return true;
}

static void _set_electromotor_rate(int left_electromotor_rate, int right_electromotor_rate) {
	UNS32 size = 0;
	int result = 0;

	union tail_electromotor_307 command;
	command.data = 0;

	ROS_ERROR("_set_electromotor_rate %hd %hd", left_electromotor_rate, right_electromotor_rate);
	command.left_electomotor_rate = (uint16_t) left_electromotor_rate;
	command.right_electromotor_rate = (uint16_t) right_electromotor_rate;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5002, 0x0, &command.data, &size, 0);
}

static void _set_electromotor_startstop(int left_startstop, int right_startstop) {
	int result = 0;
	union tail_electromotor_207 command;
	command.data = 0;

	command.left_electromotor_startstop = left_startstop ? ELECTROMOTOR_START : ELECTROMOTOR_STOP; // start
	command.right_electromotor_startstop = right_startstop ? ELECTROMOTOR_START : ELECTROMOTOR_STOP; // start

	UNS32 size = sizeof(command.data);
	// fill our super variable
	result = writeLocalDict(&actuators_Data, 0x5001, 0x0, &command.data, &size, 0);
}

static void _set_electromotor_angle(double left_electromotor_angle_X,
		double left_electromotor_angle_Y,
		double right_electromotor_angle_X,
		double right_electromotor_angle_Y) {
	UNS32 size = 0;
	int result = 0;

	union tail_electromotor_407 command;
	command.data = 0;

	ROS_ERROR("_set_electromotor_angle %f %f %f %f", left_electromotor_angle_X, left_electromotor_angle_Y, right_electromotor_angle_X, right_electromotor_angle_Y);
	command.tail_engine_rotation_X_angle = (int16_t) left_electromotor_angle_X * 100;
	command.tail_engine_rotation_Y_angle = (int16_t) left_electromotor_angle_Y * 100;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5003, 0x0, &command.data, &size, 0);
}

bool set_left_main_engine_relay_state(actuators::SetEngineRelayStateRequest &request, actuators::SetEngineRelayStateResponse &resp) {
	UNS32 size = 0;
	int result = 0;

	union left_main_engine_312 command;
	command.data = 0;

	ROS_ERROR("_set_left_main_engine_relay_state "
			"ECU %hhu AUX FUEL %hhu ELECTROMOTOR_START %hhu IGNITION1 %hhu IGNITION2 %hhu MAGNETIC_VALVE %hhu MAIN_FUEL_PUMP %hhu SERVO %hhu",
			request.ECU,
			request.aux_fuel_pump,
			request.electromotor_start,
			request.ignition1,
			request.ignition2,
			request.magnetic_valve,
			request.main_fuel_pump,
			request.servo
			);

	command.control.ECU = request.ECU;
	command.control.aux_fuel_pump = request.aux_fuel_pump;
	command.control.electromotor_start = request.electromotor_start;
	command.control.ignition_1 = request.ignition1;
	command.control.ignition_2 = request.ignition2;
	command.control.magnetic_valve = request.magnetic_valve;
	command.control.main_fuel_pump = request.main_fuel_pump;
	command.control.servo = request.servo;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5014, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x14);

	return true;
}

bool set_right_main_engine_relay_state(actuators::SetEngineRelayStateRequest &request, actuators::SetEngineRelayStateResponse &resp) {
	UNS32 size = 0;
	int result = 0;

	union right_main_engine_313 command;
	command.data = 0;

	ROS_ERROR("_set_right_main_engine_relay_state "
			"ECU %hhu AUX FUEL %hhu ELECTROMOTOR_START %hhu IGNITION1 %hhu IGNITION2 %hhu MAGNETIC_VALVE %hhu MAIN_FUEL_PUMP %hhu SERVO %hhu",
			request.ECU,
			request.aux_fuel_pump,
			request.electromotor_start,
			request.ignition1,
			request.ignition2,
			request.magnetic_valve,
			request.main_fuel_pump,
			request.servo
			);

	command.control.ECU = request.ECU;
	command.control.aux_fuel_pump = request.aux_fuel_pump;
	command.control.electromotor_start = request.electromotor_start;
	command.control.ignition_1 = request.ignition1;
	command.control.ignition_2 = request.ignition2;
	command.control.magnetic_valve = request.magnetic_valve;
	command.control.main_fuel_pump = request.main_fuel_pump;
	command.control.servo = request.servo;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5015, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x15);

	return true;
}

static void _set_left_main_engine_rotation_angle(float angle) {
	UNS32 size = 0;
	int result = 0;

	union left_main_engine_rotation_20A command;
	command.data = 0;

	ROS_ERROR("_set_left_main_engine_rotation_angle %f", angle);
	command.left_main_engine_rotation_angle = (int16_t) angle * 100;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x500E, 0x0, &command.data, &size, 0);
}

static void _set_right_main_engine_rotation_angle(float angle) {
	UNS32 size = 0;
	int result = 0;

	union right_main_engine_rotation_20B command;
	command.data = 0;

	ROS_ERROR("_set_right_main_engine_rotation_angle %f", angle);
	command.right_main_engine_rotation_angle = (int16_t) angle * 100;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x500F, 0x0, &command.data, &size, 0);
}

static bool set_electromotor_onoff(actuators::SetElectromotorsOnOffRequest &request, actuators::SetElectromotorsOnOffResponse &response) {

	ROS_ERROR("_set_electromotor_startstop %hhd %hhd", request.left_electromotor_on, request.right_electromotor_on);

	_set_electromotor_startstop(request.left_electromotor_on, request.right_electromotor_on);
	enqueue_PDO(0x1);

	return true;
}

volatile int left_engine_startstop_command_sent = 0;
volatile int right_engine_startstop_command_sent = 0;

static bool start_left_engine(actuators::StartEngineRequest &request, actuators::StartEngineResponse &response) {
	UNS32 size = 0;
	int result = 0;

	union left_main_engine_208_request command;
	command.data = 0;

	ROS_ERROR("_start_left_engine");
	command.start_stop = (uint8_t) 0xA0;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5010, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x10);

	left_engine_startstop_command_sent = 1;

	return true;
}

static bool stop_left_engine(actuators::StopEngineRequest &request, actuators::StopEngineResponse &response) {
	UNS32 size = 0;
	int result = 0;

	union left_main_engine_208_request command;
	command.data = 0;

	ROS_ERROR("_stop_left_engine");
	command.start_stop = (uint8_t) 0x55;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5010, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x10);
	left_engine_startstop_command_sent = 1;

	return true;
}

static bool start_right_engine(actuators::StartEngineRequest &request, actuators::StartEngineResponse &response) {
	UNS32 size = 0;
	int result = 0;

	union right_main_engine_209_request command;
	command.data = 0;

	ROS_ERROR("_start_right_engine");
	command.start_stop = (uint8_t) 0xA0;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5011, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x11);
	right_engine_startstop_command_sent = 1;

	return true;
}

volatile int switch_helium_valve_command_sent = 0;
static bool switch_helium_valve(actuators::SwitchHeliumValveRequest &request, actuators::SwitchHeliumValveResponse &response) {
	UNS32 size = 0;
	int result = 0;

	union helium_valve_20e_request command;
	command.data = 0;

	ROS_ERROR("switch_helium_valve %hhd", request.open);
	command.start_stop = (uint8_t) request.open ? 0xAA : 0x55;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5018, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x18);
	switch_helium_valve_command_sent = 1;

	return true;
}

static bool stop_right_engine(actuators::StopEngineRequest &request, actuators::StopEngineResponse &response) {
	UNS32 size = 0;
	int result = 0;

	union right_main_engine_209_request command;
	command.data = 0;

	ROS_ERROR("_stop_right_engine");
	command.start_stop = (uint8_t) 0x55;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5011, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x11);

	right_engine_startstop_command_sent = 1;

	return true;
}

static void _set_left_motor_control(uint8_t control_regime, float control) {
	UNS32 size = 0;
	int result = 0;

	union left_main_engine_308 command;
	command.data = 0;

	ROS_ERROR("_set_left_motor_control %hhd %s %f", control_regime, control_regime ? "throttle" : "rate", control);
	switch (control_regime) {
	case 0: // RATE
		command.regime = 0xAA;
		command.rate = (UNS16) control;
		break;
	case 1: // THROTTLE
		command.regime = 0x55;
		command.throttle = (UNS16) control * 10.0;
		break;
	}

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5012, 0x0, &command.data, &size, 0);
}

static void _set_right_motor_control(uint8_t control_regime, float control) {
	UNS32 size = 0;
	int result = 0;

	union right_main_engine_309 command;
	command.data = 0;

	ROS_ERROR("_set_right_motor_control %hhd %s %f", control_regime, control_regime ? "throttle" : "rate", control);
	switch (control_regime) {
	case 0: // RATE
		command.regime = 0xAA;
		command.rate = (UNS16) control;
		break;
	case 1: // THROTTLE
		command.regime = 0x55;
		command.throttle = (UNS16) control * 10.0;
		break;
	}

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5013, 0x0, &command.data, &size, 0);
}

bool set_generator_control(actuators::SetGeneratorControlRequest &request, actuators::SetGeneratorControlResponse &response) {
	UNS32 size = 0;
	int result = 0;

	union generator_205 command;
	command.data = 0;

	ROS_ERROR("set_generator_control %hhd %hhd", request.left_generator_start, request.right_generator_start);
	switch (request.left_generator_start) {
	case 0:
		command.left_generator_start = 0xb1;
		break;
	case 1:
		command.left_generator_start = 0xa1;
		break;
	}

	switch (request.right_generator_start) {
	case 0:
		command.right_generator_start = 0xb1;
		break;
	case 1:
		command.right_generator_start = 0xa1;
		break;
	}

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5017, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x17);

	return true;
}

UNS8 flag_helper(uint8_t flag) {
	if (flag == 0)
		return 0x5;

	if (flag == 1)
		return 0xA;

	return 0x0;
}

bool set_power_relay(actuators::SetPowerRelayRequest &request, actuators::SetPowerRelayResponse &response) {
	UNS32 size = 0;
	int result = 0;

	union power_distribution_relay_305 command;
	command.data = 0;

	ROS_ERROR("set_power_relay CONTROL_TYPE %hhd BACKUP_AND_FAILSAFE %hhd BOTTOM_SIGNAL_LIGHTS %hhd TOP_SIGNAL_LIGHTS %hhd LEFT_POWER_EQUIPMENT %hhd "
			"RIGHT_POWER_EQUIPMENT %hhd LEFT_PRESSURE_CONTROL %hhd RIGHT_PRESSURE_CONTROL %hhd LOAD_POWER %hhd TAIL_270v %hhd TAIL_28v %hhd ",
			request.control_type,
			request.backup_and_failsafe_battery,
			request.bottom_signal_lights,
			request.top_signal_lights,
			request.left_power_equipment,
			request.right_power_equipment,
			request.left_pressure_control,
			request.right_pressure_control,
			request.load_power,
			request.tail_270v_power_distribution,
			request.tail_28v_power_distribution
	);

	switch (request.control_type) {
	case 0:
		command.control_type = 0xAF; // automatic
		break;
	case 1:
		command.control_type = 0xBF; // manual
		break;
	}

	command.backup_and_failsafe_battery = flag_helper(request.backup_and_failsafe_battery);
	command.bottom_signal_lights = flag_helper(request.bottom_signal_lights);

	command.left_power_equipment = flag_helper(request.left_power_equipment);
	command.right_power_equipment = flag_helper(request.right_power_equipment);

	command.left_pressure_control = flag_helper(request.left_pressure_control);
	command.right_pressure_control = flag_helper(request.right_pressure_control);

	command.load_power = flag_helper(request.load_power);
	command.top_signal_lights = flag_helper(request.top_signal_lights);

	command.tail_270v_power_distribution = flag_helper(request.tail_270v_power_distribution);
	command.tail_28v_power_distribution = flag_helper(request.tail_28v_power_distribution);

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5016, 0x0, &command.data, &size, 0);
	enqueue_PDO(0x16);

	return true;
}

static ros::Publisher motorControlPublisher;

void reportMotorsControl(actuators::SetMotorsControlRequest &req) {
	actuators::MotorsControl state;
	state.left_electromotor_rate = req.left_electromotor_rate;
	state.left_electromotors_servo_anglex = req.left_electromotors_servo_anglex;
	state.left_electromotors_servo_angley = req.left_electromotors_servo_angley;
	state.left_electromotors_servo_fix = req.left_electromotors_servo_fix;

	state.right_electromotor_rate = req.right_electromotor_rate;
	state.right_electromotors_servo_anglex = req.right_electromotors_servo_anglex;
	state.right_electromotors_servo_angley = req.right_electromotors_servo_angley;
	state.right_electromotors_servo_fix = req.right_electromotors_servo_fix;

	state.left_engine_control = req.left_engine_control;
	state.left_engine_control_regime = req.left_engine_control_regime;
	state.left_engine_servo_angle = req.left_engine_servo_angle;
	state.left_engine_servo_fix = req.left_engine_servo_fix;

	state.right_engine_control = req.right_engine_control;
	state.right_engine_control_regime = req.right_engine_control_regime;
	state.right_engine_servo_angle = req.right_engine_servo_angle;
	state.right_engine_servo_fix = req.right_engine_servo_fix;

	motorControlPublisher.publish(state);

	QUEUE_MATLAB_NOTIFY_VAR(req.left_electromotor_rate, "left_electromotor_rate %f");
	QUEUE_MATLAB_NOTIFY_VAR(req.left_electromotors_servo_anglex, "left_electromotors_servo_anglex %f");
	QUEUE_MATLAB_NOTIFY_VAR(req.left_electromotors_servo_angley, "left_electromotors_servo_angley %f");

	QUEUE_MATLAB_NOTIFY_VAR(req.right_electromotor_rate, "right_electromotor_rate %f");
	QUEUE_MATLAB_NOTIFY_VAR(req.right_electromotors_servo_anglex, "right_electromotors_servo_anglex %f");
	QUEUE_MATLAB_NOTIFY_VAR(req.right_electromotors_servo_angley, "right_electromotors_servo_angley %f");

	QUEUE_MATLAB_NOTIFY_VAR(req.left_engine_control_regime, "left_engine_control_regime %hhd");
	QUEUE_MATLAB_NOTIFY_VAR(req.left_engine_control, "left_engine_control %f");
	QUEUE_MATLAB_NOTIFY_VAR(req.left_engine_servo_angle, "left_engine_servo_angle %f");

	QUEUE_MATLAB_NOTIFY_VAR(req.right_engine_control_regime, "right_engine_control_regime %hhd");
	QUEUE_MATLAB_NOTIFY_VAR(req.right_engine_control, "right_engine_control %f");
	QUEUE_MATLAB_NOTIFY_VAR(req.right_engine_servo_angle, "right_engine_servo_angle %f");
}

static bool set_motors_control(actuators::SetMotorsControlRequest &req,
		actuators::SetMotorsControlResponse &resp) {
	//LOG_TRACE;
	static int electromotor_engine_on = 0;

	static struct timeval electromotor_rate_tv = {0, 0};
	static struct timeval electromotor_angle_tv = {0, 0};

	struct timeval now_tv;
	long long diff = 0;

	gettimeofday(&now_tv, NULL);

	diff = now_tv.tv_sec * 1000 * 1000L + now_tv.tv_usec - (electromotor_rate_tv.tv_sec * 1000 * 1000L + electromotor_rate_tv.tv_usec);

	if (diff > 1500 * 1000) {
		_set_electromotor_rate(req.left_electromotor_rate, req.right_electromotor_rate);
		enqueue_PDO(0x2);

		gettimeofday(&electromotor_rate_tv, NULL);
	} else {
		ROS_ERROR("too fast electromotor_rate, skipping");
	}

	_set_electromotor_angle(req.left_electromotors_servo_anglex, req.left_electromotors_servo_angley,
			req.right_electromotors_servo_anglex, req.right_electromotors_servo_angley);
	enqueue_PDO(0x3);

	ROS_ERROR("i'm motors control");




	_set_left_main_engine_rotation_angle(req.left_engine_servo_angle);
	enqueue_PDO(0x0E);
	_set_right_main_engine_rotation_angle(req.right_engine_servo_angle);
	enqueue_PDO(0x0F);

	_set_left_motor_control(req.left_engine_control_regime, req.left_engine_control);
	enqueue_PDO(0x12);

	_set_right_motor_control(req.right_engine_control_regime, req.right_engine_control);
	enqueue_PDO(0x13);

	reportMotorsControl(req);

	return true;
}

bool set_control_surface(actuators::SetControlSurfaceRequest &request, actuators::SetControlSurfaceResponse &resp) {
	UNS32 size = 0;
	int result = 0;

	union control_surface_206 command;

	ROS_ERROR("set_control_surface angle_X %f angle_Y %f", request.angle_X, request.angle_Y);

	command.angle_X = (INTEGER16) request.angle_X * 100;
	command.angle_Y = (INTEGER16) request.angle_Y * 100;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x501A, 0x0, &command.data, &size, 0);

	enqueue_PDO(0x1A);

	return true;
}

/**************************************************************************
 *
 *       T           O           P          I          C           S
 *
 **************************************************************************
 */

static ros::Publisher leftBallonetStatePublisher;
static ros::Publisher rightBallonetStatePublisher;

static ros::Publisher leftBallonetStateExtPublisher;
static ros::Publisher rightBallonetStateExtPublisher;

static ros::Publisher leftElectromotorStatePublisher;
static ros::Publisher rightElectromotorStatePublisher;

static ros::Publisher leftEngineStatePublisher;
static ros::Publisher rightEngineStatePublisher;

static ros::Publisher leftEngineStateExtPublisher;
static ros::Publisher rightEngineStateExtPublisher;

static ros::Publisher leftEngineServoStatePublisher;
static ros::Publisher rightEngineServoStatePublisher;

static ros::Publisher leftEngineServoStateExtPublisher;
static ros::Publisher rightEngineServoStateExtPublisher;

static ros::Publisher electromotorsStatePublisher;
static ros::Publisher electromotorsStateExtPublisher;

static ros::Publisher powerSystemStatePublisher;

static ros::Publisher heliumValveStatePublisher;

static ros::Publisher remoteControlStatePublisher;

static ros::Publisher controlSurfaceStatePublisher;

actuators::EngineState getLeftMainEngineState(struct actuators_model *model) {
	actuators::EngineState engineState;
	actuators::EngineStateExt engineStateExt;

	engineState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	engineState.cylinder_temperature1 = model->left_main_engine_cylinder_1_temperature;
	engineState.cylinder_temperature2 = model->left_main_engine_cylinder_2_temperature;
	engineState.exhaust_gas_temperature = model->left_main_engine_exhaust_temperature;
	engineState.fuel_level_main_tank = model->left_main_engine_fuel_level;
	engineState.fuel_level_aux_tank = model->left_main_engine_aux_fuel_level;
	engineState.throttle1_angle = model->left_main_engine_throttle_1_angle;
	engineState.throttle2_angle = model->left_main_engine_throttle_2_angle;
	engineState.rate = model->left_main_engine_rate;

	engineStateExt.header.stamp = ros::Time::now();
	engineStateExt.ECU = model->left_main_engine_relay_state.ECU;
	engineStateExt.aux_fuel_pump = model->left_main_engine_relay_state.aux_fuel_pump;
	engineStateExt.electromotor_start = model->left_main_engine_relay_state.electromotor_start;
	engineStateExt.ignition_1 = model->left_main_engine_relay_state.ignition_1;
	engineStateExt.ignition_2 = model->left_main_engine_relay_state.ignition_2;
	engineStateExt.magnetic_valve = model->left_main_engine_relay_state.magnetic_valve;
	engineStateExt.main_fuel_pump = model->left_main_engine_relay_state.main_fuel_pump;
	engineStateExt.servo = model->left_main_engine_relay_state.servo;

	engineState.relay_state = engineStateExt;
	UNLOCK_MODEL();

	return engineState;
}

actuators::EngineState getRightMainEngineState(struct actuators_model *model) {
	actuators::EngineState engineState;
	actuators::EngineStateExt engineStateExt;

	engineState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	engineState.cylinder_temperature1 = model->right_main_engine_cylinder_1_temperature;
	engineState.cylinder_temperature2 = model->right_main_engine_cylinder_2_temperature;
	engineState.exhaust_gas_temperature = model->right_main_engine_exhaust_temperature;
	engineState.fuel_level_main_tank = model->right_main_engine_fuel_level;
	engineState.fuel_level_aux_tank = model->right_main_engine_aux_fuel_level;
	engineState.throttle1_angle = model->right_main_engine_throttle_1_angle;
	engineState.throttle2_angle = model->right_main_engine_throttle_2_angle;
	engineState.rate = model->right_main_engine_rate;

	engineStateExt.header.stamp = ros::Time::now();
	engineStateExt.ECU = model->right_main_engine_relay_state.ECU;
	engineStateExt.aux_fuel_pump = model->right_main_engine_relay_state.aux_fuel_pump;
	engineStateExt.electromotor_start = model->right_main_engine_relay_state.electromotor_start;
	engineStateExt.ignition_1 = model->right_main_engine_relay_state.ignition_1;
	engineStateExt.ignition_2 = model->right_main_engine_relay_state.ignition_2;
	engineStateExt.magnetic_valve = model->right_main_engine_relay_state.magnetic_valve;
	engineStateExt.main_fuel_pump = model->right_main_engine_relay_state.main_fuel_pump;
	engineStateExt.servo = model->right_main_engine_relay_state.servo;

	engineState.relay_state = engineStateExt;
	UNLOCK_MODEL();

	return engineState;
}

actuators::BallonetState getLeftBallonetState(struct actuators_model *model) {

	actuators::BallonetState leftBallonet;

	leftBallonet.header.stamp = ros::Time::now();

	LOCK_MODEL();
	leftBallonet.pressureDiff1 = model->left_ballonet_pressure_1;
	leftBallonet.pressureDiff2 = model->left_ballonet_pressure_2;
	leftBallonet.linear_valve_resistance = model->left_ballonet_linear_valve_resistance;
	switch (model->left_ballonet_lights_state) {
	case 0x55:
		leftBallonet.light = 0;
		break;
	case 0xAA:
		leftBallonet.light = 1;
		break;
	}

	switch (model->left_ballonet_regime) {
	case 0x51:
		leftBallonet.manual_regime = true;
		break;
	default:
		leftBallonet.manual_regime = false;
		break;
	}

	switch (model->left_ballonet_control) {
	case 0:
		leftBallonet.fan1 = 0;
		leftBallonet.fan2 = 0;
		leftBallonet.valve = 0;
		break;
	case 0x10:
		leftBallonet.fan1 = 0;
		leftBallonet.fan2 = 0;
		leftBallonet.valve = 1;

		break;
	case 0x11:
		leftBallonet.fan1 = 1;
		leftBallonet.fan2 = 0;
		leftBallonet.valve = 0;

		break;
	case 0x12:
		leftBallonet.fan1 = 0;
		leftBallonet.fan2 = 1;
		leftBallonet.valve = 0;

		break;
	case 0x13:
		leftBallonet.fan1 = 1;
		leftBallonet.fan2 = 1;
		leftBallonet.valve = 0;

		break;
	}
	UNLOCK_MODEL();

	return leftBallonet;
}

actuators::BallonetState getRightBallonetState(struct actuators_model *model) {

	actuators::BallonetState rightBallonet;

	rightBallonet.header.stamp = ros::Time::now();

	LOCK_MODEL();
	rightBallonet.pressureDiff1 = model->right_ballonet_pressure_1;
	rightBallonet.pressureDiff2 = model->right_ballonet_pressure_2;
	rightBallonet.linear_valve_resistance = model->right_ballonet_linear_valve_resistance;
	switch (model->right_ballonet_lights_state) {
	case 0x55:
		rightBallonet.light = 0;
		break;
	case 0xAA:
		rightBallonet.light = 1;
		break;
	}

	switch (model->right_ballonet_regime) {
		case 0x51:
			rightBallonet.manual_regime = true;
			break;
		default:
			rightBallonet.manual_regime = false;
			break;
	}

	switch (model->right_ballonet_control) {
	case 0:
		rightBallonet.fan1 = 0;
		rightBallonet.fan2 = 0;
		rightBallonet.valve = 0;
		break;
	case 0x10:
		rightBallonet.fan1 = 0;
		rightBallonet.fan2 = 0;
		rightBallonet.valve = 1;

		break;
	case 0x11:
		rightBallonet.fan1 = 1;
		rightBallonet.fan2 = 0;
		rightBallonet.valve = 0;

		break;
	case 0x12:
		rightBallonet.fan1 = 0;
		rightBallonet.fan2 = 1;
		rightBallonet.valve = 0;

		break;
	case 0x13:
		rightBallonet.fan1 = 1;
		rightBallonet.fan2 = 1;
		rightBallonet.valve = 0;

		break;
	}
	UNLOCK_MODEL();

	return rightBallonet;
}

actuators::EngineServoState getLeftMainEngineServoState(struct actuators_model *model) {
	actuators::EngineServoState servoState;

	servoState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	servoState.angle = model->left_main_engine_rotation_angle;
	servoState.voltage = model->left_main_engine_rotation_voltage;
	servoState.current = model->left_main_engine_rotation_current;
	UNLOCK_MODEL();

	return servoState;
}

actuators::EngineServoState getRightMainEngineServoState(struct actuators_model *model) {
	actuators::EngineServoState servoState;

	servoState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	servoState.angle = model->right_main_engine_rotation_angle;
	servoState.voltage = model->right_main_engine_rotation_voltage;
	servoState.current = model->right_main_engine_rotation_current;
	UNLOCK_MODEL();

	return servoState;
}

actuators::PowerSystemState getPowerSystemState(struct actuators_model *model) {
	actuators::PowerSystemState powerSystemState;

	powerSystemState.header.stamp = ros::Time::now();

	// 0.1A
	powerSystemState.left_main_engine_generator_output_current = model->left_main_engine_generator_output_current;

	// 0.1V
	powerSystemState.left_main_engine_generator_output_voltage = model->left_main_engine_generator_output_voltage;

	// 0.1A
	powerSystemState.right_main_engine_generator_output_current = model->right_main_engine_generator_output_current;

	// 0.1V
	powerSystemState.right_main_engine_generator_output_voltage = model->right_main_engine_generator_output_voltage;

	// 0.1A
	powerSystemState.left_ballonet_control_current = model->left_ballonet_control_current;

	// 0.1A
	powerSystemState.right_ballonet_control_current = model->right_ballonet_control_current;

	// 0,1A
	powerSystemState.power_section_28v_power_equipment_current = model->power_section_28v_power_equipment_current;

	// 0,1A
	powerSystemState.tail_section_28v_power_equipment_current = model->tail_section_28v_power_equipment_current;

	// 0.1V
	powerSystemState.bus_backup_28v_voltage = model->bus_backup_28v_voltage;

	// 0,1V
	powerSystemState.failsafe_28v_voltage = model->failsafe_28v_voltage;

	// 0.1V
	powerSystemState.backup_battery_voltage = model->backup_battery_voltage;

	// 0,1V
	powerSystemState.failsafe_battery_voltage = model->failsafe_battery_voltage;

	// 0.1A
	powerSystemState.backup_battery_charge_current = model->backup_battery_charge_current;

	// 0.1A
	powerSystemState.backup_battery_discharge_current = model->backup_battery_discharge_current;

	// 1C
	powerSystemState.backup_battery_monoblock_temperature = model->backup_battery_monoblock_temperature;

	// примечание 1
	actuators::BatteryBackupState batteryBackupState;
	batteryBackupState.header.stamp = ros::Time::now();

	batteryBackupState.battery1_failure_code = model->backup_battery_failure_state.battery1_failure_code;
	batteryBackupState.battery2_failure_code = model->backup_battery_failure_state.battery2_failure_code;
	batteryBackupState.battery3_failure_code = model->backup_battery_failure_state.battery3_failure_code;

	batteryBackupState.battery1_failure_flag = model->backup_battery_failure_state.battery1_failure_flag;
	batteryBackupState.battery2_failure_flag = model->backup_battery_failure_state.battery2_failure_flag;
	batteryBackupState.battery3_failure_flag = model->backup_battery_failure_state.battery3_failure_flag;

	powerSystemState.backup_battery_failure_state = batteryBackupState;

	// 0.1A
	powerSystemState.failsafe_battery_charge_current = model->failsafe_battery_charge_current;

	// 0.1A
	powerSystemState.failsafe_battery_discharge_current = model->failsafe_battery_discharge_current;

	// 1C
	powerSystemState.failsafe_battery_monoblock_temperature = model->failsafe_battery_monoblock_temperature;

	// примечание 2
	actuators::BatteryFailsafeState batteryFailsafeState;
	batteryFailsafeState.header.stamp = ros::Time::now();

	batteryFailsafeState.failsafe_battery1_failure_flag = model->failsafe_battery_failure_state.failsafe_battery1_failure_flag;
	batteryFailsafeState.failsafe_battery2_failure_flag = model->failsafe_battery_failure_state.failsafe_battery2_failure_flag;

	batteryFailsafeState.failsafe_battery1_failure_code = model->failsafe_battery_failure_state.failsafe_battery1_failure_code;
	batteryFailsafeState.failsafe_battery2_failure_code = model->failsafe_battery_failure_state.failsafe_battery2_failure_code;

	powerSystemState.failsafe_battery_failure_state = batteryFailsafeState;

	// 0.1A
	powerSystemState.left_charge_device_output_current = model->left_charge_device_output_current;

	// reserved
	powerSystemState.left_charge_device_state = model->left_charge_device_state;

	// 0.1A
	powerSystemState.right_charge_device_output_current = model->right_charge_device_output_current;

	// reserved
	powerSystemState.right_charge_device_state = model->right_charge_device_state;

	// примечание 3
	actuators::PowerDistributionRelayState powerDistributionRelayState;
	powerDistributionRelayState.header.stamp = ros::Time::now();

	powerDistributionRelayState.both_battery_power = model->distribution_relay_state.both_battery_power;
	powerDistributionRelayState.bottom_signal_lights = model->distribution_relay_state.bottom_signal_lights;
	powerDistributionRelayState.top_signal_lights = model->distribution_relay_state.top_signal_lights;
	powerDistributionRelayState.left_power_equipment = model->distribution_relay_state.left_power_equipment;
	powerDistributionRelayState.right_power_equipment = model->distribution_relay_state.right_power_equipment;
	powerDistributionRelayState.tail_28v_equipment = model->distribution_relay_state.tail_28v_equipment;
	powerDistributionRelayState.left_pressure_control = model->distribution_relay_state.left_pressure_control;
	powerDistributionRelayState.right_pressure_control = model->distribution_relay_state.right_pressure_control;
	powerDistributionRelayState.load = model->distribution_relay_state.load;
	powerDistributionRelayState.reserved = model->distribution_relay_state.reserved;

	powerSystemState.distribution_relay_state = powerDistributionRelayState;

	// 0.1A
	powerSystemState.load_equipment_28v_current = model->load_equipment_28v_current;

	// примечание 4
	actuators::PowerDistributionLineFailure powerDistributionLineFailure;
	powerDistributionLineFailure.header.stamp = ros::Time::now();

	powerDistributionLineFailure.bottom_signal_lights_failure = model->distribution_lines_failure_state.bottom_signal_lights_failure;
	powerDistributionLineFailure.top_signal_lights_failure = model->distribution_lines_failure_state.top_signal_lights_failure;
	powerDistributionLineFailure.left_power_equipment_failure = model->distribution_lines_failure_state.left_power_equipment_failure;
	powerDistributionLineFailure.right_power_equipment_failure = model->distribution_lines_failure_state.right_power_equipment_failure;
	powerDistributionLineFailure.tail_equipment_failure = model->distribution_lines_failure_state.tail_equipment_failure;

	powerSystemState.distribution_lines_failure_state = powerDistributionLineFailure;

	return powerSystemState;
}

actuators::HeliumValveState getHeliumValveState(struct actuators_model *model) {
	actuators::HeliumValveState state;

	state.header.stamp = ros::Time::now();

	state.helium_valve_open = (uint8_t) model->helium_valve_open;

	return state;
}

actuators::ElectromotorState getLeftElectromotorState(struct actuators_model *model) {
	actuators::ElectromotorState state;

	state.header.stamp = ros::Time::now();

	state.rate = model->left_electromotor_rate;

	return state;
}


actuators::ElectromotorState getRightElectromotorState(struct actuators_model *model) {
	actuators::ElectromotorState state;

	state.header.stamp = ros::Time::now();

	state.rate = model->right_electromotor_rate;

	return state;
}

actuators::ElectromotorServoState getLeftElectromotorServoState(struct actuators_model *model) {
	actuators::ElectromotorServoState state;

	state.anglex = model->left_electromotor_angle_X;
	state.angley = model->left_electromotor_angle_Y;
	/* FIXME IMHO there is no fix for electromotor servos */
	state.fix = false;

	return state;
}

actuators::ElectromotorServoState getRightElectromotorServoState(struct actuators_model *model) {
	actuators::ElectromotorServoState state;

	state.anglex = model->right_electromotor_angle_X;
	state.angley = model->right_electromotor_angle_Y;
	/* FIXME IMHO there is no fix for electromotor servos */
	state.fix = false;

	return state;
}

actuators::ElectromotorsState getElectromotorsState(struct actuators_model *model) {
	actuators::ElectromotorsState electromotorsState;

	electromotorsState.header.stamp = ros::Time::now();

	electromotorsState.left_em = getLeftElectromotorState(model);
	electromotorsState.right_em = getRightElectromotorState(model);
	electromotorsState.left_em_servo = getLeftElectromotorServoState(model);
	electromotorsState.right_em_servo = getRightElectromotorServoState(model);

	return electromotorsState;
}

actuators::ElectromotorsStateExt getElectromotorsStateExt(struct actuators_model *model) {
	actuators::ElectromotorsStateExt state;
	state.header.stamp = ros::Time::now();
	state.basic_state = getElectromotorsState(model);

	state.left_electromotor_current = model->left_electromotor_current;
	state.right_electromotor_current = model->right_electromotor_current;

	state.left_electromotor_voltage = model->left_electromotor_voltage;
	state.right_electromotor_voltage = model->right_electromotor_voltage;

	state.left_electromotor_temperature = model->left_electromotor_temperature;
	state.left_electromotor_servo_temperature = model->left_electromotor_rotation_temperature;

	state.right_electromotor_temperature = model->right_electromotor_temperature;
	state.right_electromotor_servo_temperature = model->right_electromotor_rotation_temperature;

	return state;
}

actuators::RemoteControlState getRemoteControlState(struct actuators_model *model) {
	actuators::RemoteControlState state;

	state.header.stamp = ros::Time::now();

	state.remote_control_allowed = model->remote_control_allowed_state;
	state.remote_control_enabled = model->remote_control_enabled;

	return state;
}

actuators::ControlSurfaceState getControlSurfaceState(struct actuators_model *model) {
	actuators::ControlSurfaceState state;

	state.header.stamp = ros::Time::now();

	state.control_surface_1_current = model->control_surface_1_current;
	state.control_surface_2_current = model->control_surface_2_current;
	state.control_surface_3_current = model->control_surface_3_current;
	state.control_surface_4_current = model->control_surface_4_current;

	state.control_surface_5_current = model->control_surface_5_current;
	state.control_surface_6_current = model->control_surface_6_current;
	state.control_surface_7_current = model->control_surface_7_current;
	state.control_surface_8_current = model->control_surface_8_current;

	state.left_control_surface_X = model->left_control_surface_X;
	state.left_control_surface_Y = model->left_control_surface_Y;

	state.right_control_surface_X = model->right_control_surface_X;
	state.right_control_surface_Y = model->right_control_surface_Y;

	return state;
}

void report_topics(void) {
	struct actuators_model *model;
	int i = 0;

	model = &MODEL;

	remoteControlStatePublisher.publish(getRemoteControlState(model));

	for (i = 0; i < callback_no; i++) {
		if (!sem_trywait(&callbacks[i].semaphore)) {
			switch (callbacks[i].index) {
			case 0x4000:
				break;
			case 0x4001:
				break;
			case 0x4002:
				break;
			case 0x4003:
				break;
			case 0x4004:
				break;
			case 0x4005:
				powerSystemStatePublisher.publish(getPowerSystemState(model));
				break;
			case 0x4006:
				controlSurfaceStatePublisher.publish(getControlSurfaceState(model));
				break;
			case 0x4007:
				electromotorsStateExtPublisher.publish(getElectromotorsStateExt(model));
				break;
			case 0x4008:
				leftEngineStatePublisher.publish(getLeftMainEngineState(model));
				break;
			case 0x4009:
				rightEngineStatePublisher.publish(getRightMainEngineState(model));
				break;
			case 0x400a:
				leftEngineServoStatePublisher.publish(getLeftMainEngineServoState(model));
				break;
			case 0x400b:
				rightEngineServoStatePublisher.publish(getRightMainEngineServoState(model));
				break;
			case 0x400c:
				leftBallonetStatePublisher.publish(getLeftBallonetState(model));
				break;
			case 0x400d:
				rightBallonetStatePublisher.publish(getRightBallonetState(model));
				break;
			case 0x400e:
				heliumValveStatePublisher.publish(getHeliumValveState(model));
				break;
			}
		}
	}
}

extern sig_atomic_t exit_flag;

void *ros_main(void *data) {
	struct actuators_ros_settings *settings = (struct actuators_ros_settings	 *) data;

	fprintf(stderr, "starting ROS...\n");

	ros::init(settings->argc, settings->argv, "actuators", 0);

	ros::NodeHandle nh;

	leftBallonetStatePublisher = nh.advertise<actuators::BallonetState>("/left_ballonet_state", 10);
	rightBallonetStatePublisher = nh.advertise<actuators::BallonetState>("/right_ballonet_state", 10);

	leftBallonetStateExtPublisher = nh.advertise<actuators::BallonetStateExt>("/left_ballonet_state_ext", 10);
	rightBallonetStateExtPublisher = nh.advertise<actuators::BallonetStateExt>("/right_ballonet_state_ext", 10);

	leftEngineStatePublisher = nh.advertise<actuators::EngineState>("/left_engine_state", 10);
	rightEngineStatePublisher = nh.advertise<actuators::EngineState>("/right_engine_state", 10);

	leftEngineStateExtPublisher = nh.advertise<actuators::EngineStateExt>("/left_engine_state_ext", 10);
	rightEngineStateExtPublisher = nh.advertise<actuators::EngineStateExt>("/right_engine_state_ext", 10);

	leftEngineServoStatePublisher = nh.advertise<actuators::EngineServoState>("/left_engine_servo_state", 10);
	rightEngineServoStatePublisher = nh.advertise<actuators::EngineServoState>("/right_engine_servo_state", 10);

	heliumValveStatePublisher = nh.advertise<actuators::HeliumValveState>("/helium_valve_state", 10);

	electromotorsStateExtPublisher = nh.advertise<actuators::ElectromotorsStateExt>("/electromotors_state_ext", 10);

	motorControlPublisher = nh.advertise<actuators::MotorsControl>("/motors_control", 10);

	remoteControlStatePublisher = nh.advertise<actuators::RemoteControlState>("/remote_control_state", 10);

	powerSystemStatePublisher = nh.advertise<actuators::PowerSystemState>("/power_system_state", 10);

	controlSurfaceStatePublisher = nh.advertise<actuators::ControlSurfaceState>("/control_surface_state", 10);

	set_generator_control_service = nh.advertiseService("/actuators/set_generator_control", set_generator_control);
	set_power_relay_service = nh.advertiseService("/actuators/set_power_relay", set_power_relay);

	set_motors_control_service = nh.advertiseService("/actuators/set_motors_control", set_motors_control);
	set_tail_electromotor_onoff_service = nh.advertiseService("/actuators/set_electromotor_onoff", set_electromotor_onoff);

	start_left_engine_service = nh.advertiseService("/actuators/start_left_engine", start_left_engine);
	stop_left_engine_service = nh.advertiseService("/actuators/stop_left_engine", stop_left_engine);
	set_left_engine_relay_state_service = nh.advertiseService("/actuators/set_left_engine_relay_state", set_left_main_engine_relay_state);

	start_right_engine_service = nh.advertiseService("/actuators/start_right_engine", start_right_engine);
	stop_right_engine_service = nh.advertiseService("/actuators/stop_right_engine", stop_right_engine);
	set_right_engine_relay_state_service = nh.advertiseService("/actuators/set_right_engine_relay_state", set_right_main_engine_relay_state);

	set_left_ballonet_automatic_control_parameters_service = nh.advertiseService("/actuators/set_left_ballonet_automatic_control_parameters", set_left_ballonet_automatic_control_parameters);
	set_left_ballonet_regime_service = nh.advertiseService("/actuators/set_left_ballonet_regime", set_left_ballonet_regime);
	set_left_ballonet_control_service = nh.advertiseService("/actuators/set_left_ballonet_control", set_left_ballonet_control);

	set_right_ballonet_automatic_control_parameters_service = nh.advertiseService("/actuators/set_right_ballonet_automatic_control_parameters", set_right_ballonet_automatic_control_parameters);
	set_right_ballonet_regime_service = nh.advertiseService("/actuators/set_right_ballonet_regime", set_right_ballonet_regime);
	set_right_ballonet_control_service = nh.advertiseService("/actuators/set_right_ballonet_control", set_right_ballonet_control);

	switch_helium_valve_service = nh.advertiseService("/actuators/switch_helium_valve", switch_helium_valve);

	set_remote_control_allow_service = nh.advertiseService("/actuators/set_remote_control_allow", set_remote_control_allow);

	set_control_surface_service = nh.advertiseService("/actuators/set_control_surface", set_control_surface);

	while (ros::ok() && !exit_flag) {
		report_topics();
		ros::spinOnce();
		ros::Duration(0.001).sleep();
	}

	ros::shutdown();

	ROS_ERROR("ROS thread exiting");

	return NULL;
}

