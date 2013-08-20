/*
 * actuators-ros.cpp
 *
 *  Created on: 15.12.2012
 *      Author: yury
 */
#include <ros/ros.h>
#include <ros/console.h>

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
#include "transport.h"

static ros::ServiceServer set_motors_control_service;
static ros::ServiceServer set_left_ballonet_control_service;
static ros::ServiceServer set_right_ballonet_control_service;
static ros::ServiceServer switch_helium_valve_service;

static bool set_motors_control(actuators::SetMotorsControlRequest &req,
		actuators::SetMotorsControlResponse &resp) {
	//LOG_TRACE;

	ROS_ERROR("i'm motors control");

	//LOCK_MODEL();
	MODEL.left_electromotor_rate =  req.left_electromotor_rate;
	MODEL.left_electromotor_angle_X = req.left_electromotors_servo_anglex;
	MODEL.left_electromotor_angle_Y = req.left_electromotors_servo_angley;

	MODEL.right_electromotor_rate =  req.right_electromotor_rate;
	MODEL.right_electromotor_angle_X = req.right_electromotors_servo_anglex;
	MODEL.right_electromotor_angle_Y = req.right_electromotors_servo_angley;
	enqueue_MID(0x2a); // FIXME temporary message for both electromotors
	MODEL.tail_engine_angle_X = req.left_electromotors_servo_anglex;
	MODEL.tail_engine_angle_Y = req.left_electromotors_servo_angley;
	MODEL.tail_engine_rate = req.left_electromotor_rate;
	MODEL.tail_engine_fix = req.left_electromotors_servo_fix ? MAIN_ENGINE_FIX_FORCE : MAIN_ENGINE_FIX_NONE;
	// enqueue_MID(0x06); // "tail_engine" aka left engine

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

#if 0
	snprintf(buf, 255, "left electromotor rate %f\n", req.left_electromotor_rate);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "left electromotor anglex %f\n", req.left_electromotors_servo_anglex);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "left electromotor angley %f\n", req.left_electromotors_servo_angley);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "right electromotor rate %f\n", req.right_electromotor_rate);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "right electromotor anglex %f\n", req.right_electromotors_servo_anglex);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "right electromotor angley %f\n", req.right_electromotors_servo_angley);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "left engine rate %f\n", req.left_engine_rate);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "left engine angle %f\n", req.left_engine_servo_angle);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "right engine rate %f\n", req.right_engine_rate);
	outgoing_queue.push(buf);

	snprintf(buf, 255, "right engine angle %f\n", req.right_engine_servo_angle);
	outgoing_queue.push(buf);
#endif
	return true;
}

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

#if 0
	snprintf(buf, 255, "left ballonet fan1 %s\n", req.fan1 ? "true" : "false");
	outgoing_queue.push(buf);

	snprintf(buf, 255, "left ballonet fan2 %s\n", req.fan2 ? "true" : "false");
	outgoing_queue.push(buf);

	snprintf(buf, 255, "left ballonet valve %s\n", req.valve ? "true" : "false");
	outgoing_queue.push(buf);

	resp.result = 0;

#endif

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

#if 0
	model.ballonets.right.fan1 = req.fan1;
	snprintf(buf, 255, "right ballonet fan1 %s\n", req.fan1 ? "true" : "false");
	outgoing_queue.push(buf);

	model.ballonets.right.fan2 = req.fan2;
	snprintf(buf, 255, "right ballonet fan2 %s\n", req.fan2 ? "true" : "false");
	outgoing_queue.push(buf);

	model.ballonets.right.valve = req.valve;
	snprintf(buf, 255, "right ballonet valve %s\n", req.valve ? "true" : "false");
	outgoing_queue.push(buf);
#endif

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

	model = &REPORTED_DATA;

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

	actuators::ElectromotorsState electromotorsState;

	actuators::EngineStateExt leftEngineStateExt;
	actuators::EngineStateExt rightEngineStateExt;

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

	actuators::ElectromotorServoState leftElectromotorServoState;
	actuators::ElectromotorServoState rightElectromotorServoState;

	leftElectromotorServoState.header.stamp = ros::Time::now();
	LOCK_MODEL();
	leftElectromotorServoState.anglex = model->left_electromotor_angle_X;
	leftElectromotorServoState.angley = model->left_electromotor_angle_Y;
	/* FIXME IMHO there is no fix for electromotor servos */
	leftElectromotorServoState.fix = false;;
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

void *ros_main(void *data) {
	struct actuators_ros_settings *settings = (struct actuators_ros_settings	 *) data;
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
	set_left_ballonet_control_service = nh.advertiseService("/actuators/set_left_ballonet_control", set_left_ballonet_control);
	set_right_ballonet_control_service = nh.advertiseService("/actuators/set_right_ballonet_control", set_right_ballonet_control);
	switch_helium_valve_service = nh.advertiseService("/actuators/switch_helium_valve", switch_helium_valve);

	while (ros::ok()) {
		report_topics();
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return NULL;
}

