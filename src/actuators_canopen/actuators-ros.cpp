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

#include "actuators/EngineState.h"
#include "actuators/EngineStateExt.h"
#include "actuators/EngineServoState.h"
#include "actuators/EngineServoStateExt.h"

#include "actuators/SetMotorsControl.h"
#include "actuators/SetBallonetControl.h"
#include "actuators/SwitchHeliumValve.h"

#include "model.h"
#include "canopen-data.h"
#include "canopen-util.h"

static ros::ServiceServer set_motors_control_service;
static ros::ServiceServer set_left_ballonet_control_service;
static ros::ServiceServer set_right_ballonet_control_service;
static ros::ServiceServer switch_helium_valve_service;

static void _set_electromotor_rate(int left_electromotor_rate, int right_electromotor_rate) {
	UNS32 size = 0;
	int result = 0;

	union tail_electromotor_307 command;
	command.data = 0;

	printf("_set_electromotor_rate %h %h\n", left_electromotor_rate, right_electromotor_rate);
	command.left_electomotor_rate = (uint16_t) left_electromotor_rate;
	command.right_electromotor_rate = (uint16_t) right_electromotor_rate;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5002, 0x0, &command.data, &size, 0);
}

static void _set_electromotor_startstop(int startstop) {
	int result = 0;
	union tail_electromotor_207 command;
	command.data = 0;
	command.startstop = startstop ? ELECTROMOTOR_START : ELECTROMOTOR_STOP; // start
	UNS32 size = sizeof(command.data);
	// fill our super variable
	result = writeLocalDict(&actuators_Data, 0x5001, 0x0, &command.data, &size, 0);

	if (startstop == ELECTROMOTOR_START) {
		_set_electromotor_rate(500, 500);
	}
}

static void _set_electromotor_angle(double left_electromotor_angle_X,
		double left_electromotor_angle_Y,
		double right_electromotor_angle_X,
		double right_electromotor_angle_Y) {
	UNS32 size = 0;
	int result = 0;

	union tail_electromotor_407 command;
	command.data = 0;

	command.tail_engine_rotation_X_angle = (int16_t) left_electromotor_angle_X * 100;
	command.tail_engine_rotation_Y_angle = (int16_t) left_electromotor_angle_Y * 100;

	size = sizeof(command.data);
	result = writeLocalDict(&actuators_Data, 0x5003, 0x0, &command.data, &size, 0);
}

static bool set_motors_control(actuators::SetMotorsControlRequest &req,
		actuators::SetMotorsControlResponse &resp) {
	//LOG_TRACE;
	static int electromotor_engine_on = 0;


	ROS_ERROR("i'm motors control");

	if (!electromotor_engine_on) {
		_set_electromotor_startstop(1);
		enqueue_PDO(0x1);
		enqueue_PDO(0x2);
		electromotor_engine_on = 1;
	}

	if (req.left_electromotor_rate < 500)
		req.left_electromotor_rate = 500;

	if (req.right_electromotor_rate < 500)
		req.right_electromotor_rate = 500;

	_set_electromotor_rate(req.left_electromotor_rate, req.right_electromotor_rate);
	enqueue_PDO(0x2);
	_set_electromotor_angle(req.left_electromotors_servo_anglex, req.left_electromotors_servo_angley,
			req.right_electromotors_servo_anglex, req.right_electromotors_servo_angley);
	enqueue_PDO(0x3);

#if 0
	MODEL.left_main_engine_rate = req.left_engine_rate;
	switch (MODEL.left_main_engine_rate) {
	case 0:
		MODEL.left_main_engine_startstop = COMMAND_MAIN_ENGINE_LEFT_STOP;
		break;
	default:
		MODEL.left_main_engine_startstop = COMMAND_MAIN_ENGINE_LEFT_START;
		break;
	}
	enqueue_MID(0x14); // rate + command
	MODEL.left_main_engine_angle = req.left_engine_servo_angle;
	enqueue_MID(0x0a); // angle + fix
	MODEL.right_main_engine_rate = req.right_engine_rate;
	switch (MODEL.right_main_engine_rate) {
	case 0:
		MODEL.right_main_engine_startstop = COMMAND_MAIN_ENGINE_RIGHT_STOP;
		break;
	default:
		MODEL.right_main_engine_startstop = COMMAND_MAIN_ENGINE_RIGHT_START;
		break;
	}
	enqueue_MID(0x1a); // rate + command
	MODEL.right_main_engine_angle = req.right_engine_servo_angle;
	enqueue_MID(0x0e); // angle & fix
	//UNLOCK_MODEL();
#endif

	return true;
}

#if 0
static bool set_left_ballonet_control(actuators::SetBallonetControlRequest &req,
		actuators::SetBallonetControlResponse &resp) {

//	LOG_TRACE;

	ROS_ERROR("i'm left ballonet control");

	LOCK_MODEL();
	MODEL.left_ballonet_state.ballonet_fan_1 =
			(ballonet_command) req.fan1 ?
			COMMAND_BALLONET_FORCE_OPEN : COMMAND_BALLONET_FORCE_CLOSE;
	MODEL.left_ballonet_state.ballonet_fan_2 = (ballonet_command) req.fan2 ?
			COMMAND_BALLONET_FORCE_OPEN : COMMAND_BALLONET_FORCE_CLOSE;
	MODEL.left_ballonet_state.ballonet_valve_1 = (ballonet_command) req.valve ?
			COMMAND_BALLONET_FORCE_OPEN : COMMAND_BALLONET_FORCE_CLOSE;
	enqueue_MID(0x1e); // machinery state + many other things
	// FIXME valve_2?
	UNLOCK_MODEL();

	return true;
}

static bool set_right_ballonet_control(actuators::SetBallonetControlRequest &req,
		actuators::SetBallonetControlResponse &resp) {
	//LOG_TRACE;

	ROS_ERROR("i'm right ballonet control");

	LOCK_MODEL();
	MODEL.right_ballonet_state.ballonet_fan_1 = (ballonet_command) req.fan1 ? COMMAND_BALLONET_FORCE_OPEN :
			COMMAND_BALLONET_FORCE_CLOSE;
	MODEL.right_ballonet_state.ballonet_fan_2 = (ballonet_command) req.fan2 ? COMMAND_BALLONET_FORCE_OPEN :
			COMMAND_BALLONET_FORCE_CLOSE;
	MODEL.right_ballonet_state.ballonet_valve_1 = (ballonet_command) req.valve ? COMMAND_BALLONET_FORCE_OPEN :
			COMMAND_BALLONET_FORCE_CLOSE;
	enqueue_MID(0x22);
	// FIXME valve_2?
	UNLOCK_MODEL();

	resp.result = 0;

	return true;
}

static bool switch_helium_valve(actuators::SwitchHeliumValveRequest &req,
		actuators::SwitchHeliumValveResponse &resp) {

	LOCK_MODEL();

	MODEL.left_ballonet_state.ballonet_valve_2 = req.open ?
			COMMAND_BALLONET_FORCE_OPEN :COMMAND_BALLONET_FORCE_CLOSE;
	enqueue_MID(0x1e);

	UNLOCK_MODEL();
	return true;
}
#endif

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

void report_topics(void) {
	//LOG_TRACE;

	actuators::BallonetState leftBallonet;
	actuators::BallonetState rightBallonet;

	struct actuators_model *model;

	model = &MODEL;
#if 0
	leftBallonet.header.stamp = ros::Time::now();
	LOCK_MODEL();
	leftBallonet.fan1 = model->left_ballonet_state.ballonet_fan_1;
	leftBallonet.fan2 = model->left_ballonet_state.ballonet_fan_2;
	leftBallonet.valve = model->left_ballonet_state.ballonet_valve_1;
	/* FIXME left is double, right is WORD */
	leftBallonet.pressureDiff1 = model->left_ballonet_differential_pressure_1;
	leftBallonet.pressureDiff2 = model->left_ballonet_differential_pressure_2;
	UNLOCK_MODEL();
	leftBallonetStatePublisher.publish(leftBallonet);

	rightBallonet.header.stamp = ros::Time::now();
	LOCK_MODEL();
	rightBallonet.fan1 = model->right_ballonet_state.ballonet_fan_1;
	rightBallonet.fan2 = model->right_ballonet_state.ballonet_fan_2;
	rightBallonet.valve = model->right_ballonet_state.ballonet_valve_1;
	/* FIXME right is double, right is WORD */
	rightBallonet.pressureDiff1 = model->right_ballonet_differential_pressure_1;
	rightBallonet.pressureDiff2 = model->right_ballonet_differential_pressure_2;
	UNLOCK_MODEL();
	rightBallonetStatePublisher.publish(rightBallonet);

	actuators::BallonetStateExt leftBallonetExt;
	actuators::BallonetStateExt rightBallonetExt;

	/* dunno how to fill those */

	actuators::EngineState leftEngineState;
	leftEngineState.header.stamp = ros::Time::now();
	/* fixme left is FLOAT right is WORD */
	leftEngineState.rate = model->left_main_engine_rate;
	leftEngineStatePublisher.publish(leftEngineState);

	actuators::EngineState rightEngineState;
	rightEngineState.header.stamp = ros::Time::now();
	/* fixme right is FLOAT right is WORD */
	rightEngineState.rate = model->right_main_engine_rate;
	rightEngineStatePublisher.publish(rightEngineState);
#endif
	actuators::ElectromotorsState electromotorsState;

#if 0
	actuators::EngineStateExt leftEngineStateExt;
	actuators::EngineStateExt rightEngineStateExt;
#endif
	/* dunno how to fill those */

	actuators::ElectromotorState leftElectromotorState;
	leftElectromotorState.header.stamp = ros::Time::now();
	/* FIXME left is float, right is WORD */
	leftElectromotorState.rate = model->left_electromotor_rate;
	leftElectromotorStatePublisher.publish(leftElectromotorState);

	actuators::ElectromotorState rightElectromotorState;
	rightElectromotorState.header.stamp = ros::Time::now();
	//printf("left electromotor state == %f\n", model.electromotors.left.rate);
	/* FIXME left is float, right is WORD */
	rightElectromotorState.rate = model->right_electromotor_rate;
	rightElectromotorStatePublisher.publish(rightElectromotorState);

#if 0
	actuators::EngineServoState leftEngineServoState;
	actuators::EngineServoState rightEngineServoState;

	leftEngineServoState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	leftEngineServoState.angle = model->left_main_engine_angle;
	leftEngineServoState.fix = model->left_main_engine_fix == MAIN_ENGINE_FIX_FORCE;
	UNLOCK_MODEL();
	leftEngineServoStatePublisher.publish(leftEngineServoState);

	rightEngineServoState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	rightEngineServoState.angle = model->right_main_engine_angle;
	rightEngineServoState.fix = model->right_main_engine_fix == MAIN_ENGINE_FIX_FORCE;
	UNLOCK_MODEL();
	rightEngineServoStatePublisher.publish(rightEngineServoState);
#endif

	actuators::ElectromotorServoState leftElectromotorServoState;
	actuators::ElectromotorServoState rightElectromotorServoState;

	leftElectromotorServoState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	leftElectromotorServoState.anglex = model->left_electromotor_angle_X;
	leftElectromotorServoState.angley = model->left_electromotor_angle_Y;
	/* FIXME IMHO there is no fix for electromotor servos */
	leftElectromotorServoState.fix = false;
	UNLOCK_MODEL();

	rightElectromotorServoState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	rightElectromotorServoState.anglex = model->right_electromotor_angle_X;
	rightElectromotorServoState.angley = model->right_electromotor_angle_Y;
	/* FIXME IMHO there is no fix for electromotor servos */
	rightElectromotorServoState.fix = false;
	UNLOCK_MODEL();

	electromotorsState.header.stamp = ros::Time::now();
	electromotorsState.left_em = leftElectromotorState;
	electromotorsState.right_em = rightElectromotorState;
	electromotorsState.left_em_servo = leftElectromotorServoState;
	electromotorsState.right_em_servo = rightElectromotorServoState;

	electromotorsStatePublisher.publish(electromotorsState);
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

	leftElectromotorStatePublisher = nh.advertise<actuators::ElectromotorState>("/left_electromotor_state", 10);
	rightElectromotorStatePublisher = nh.advertise<actuators::ElectromotorState>("/right_electromotor_state", 10);

	leftEngineServoStateExtPublisher = nh.advertise<actuators::ElectromotorsStateExt>("/left_engine_servo_state_ext", 10);
	rightEngineServoStateExtPublisher = nh.advertise<actuators::ElectromotorsStateExt>("/right_engine_servo_state_ext", 10);

	electromotorsStatePublisher = nh.advertise<actuators::ElectromotorsState>("/electromotors_state", 10);

	set_motors_control_service = nh.advertiseService("/actuators/set_motors_control", set_motors_control);
#if 0
	set_left_ballonet_control_service = nh.advertiseService("/actuators/set_left_ballonet_control", set_left_ballonet_control);
	set_right_ballonet_control_service = nh.advertiseService("/actuators/set_right_ballonet_control", set_right_ballonet_control);
	switch_helium_valve_service = nh.advertiseService("/actuators/switch_helium_valve", switch_helium_valve);
#endif

	while (ros::ok() && !exit_flag) {
		report_topics();
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return NULL;
}

