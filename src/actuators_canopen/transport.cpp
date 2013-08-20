/*
 * Receive and rudimentally post-process messages upon a CAN bus channel
 */

#include <assert.h>
#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdint.h>

#include <pthread.h>

#include "matlab-connector.h"
#include "actuators.h"
#include "transport.h"
#include "model.h"

/* data goes from ROS service calls to MODEL; CAN senders use MODEL */
struct actuators_model MODEL;
/* data goes from CAN to REPORTED_DATA; topics use REPORTED_DATA; */
struct actuators_model REPORTED_DATA;

std::queue<int> send_queue_MID;

pthread_mutex_t MODEL_lock;
pthread_mutex_t REPORTED_DATA_lock;
pthread_mutex_t send_queue_MID_lock;

static canHandle h;

static struct CAN_receivers RECEIVER;

/**
 * function to call CanLib
 * @param id
 * @param stat
 */
static canStatus checkCanLibCall(const char* id, canStatus stat)
{
	char buf[50];

	buf[0] = '\0';
	canGetErrorText(stat, buf, sizeof(buf));
	if (stat != canOK) {
		printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
	} else {
		printf("%s: OK\n", id);
	}

	return stat;
}

static int i = 0;
static int standard = 0, ext = 0, rtr = 0, err = 0, over = 0;
sig_atomic_t willExit = 0;

/**
 *
 * @param sig
 */
void sighand (int sig)
{

	switch (sig) {
		case SIGINT:
			willExit = 1;
			if (pthread_equal(can_thread, pthread_self())) {
					fprintf(stderr, "signal can thread exit\n");
				} else {
					fprintf(stderr, "signal 	main thread exit\n");
				}
			break;
	}
}

/**
 *
 * @param message
 * @param frame_count
 * @param text_size
 */
void send_nop(union CAN_logical_message *message, uint8_t *frame_count, uint16_t *text_size) {
	printf("send_nop called\n");
}

/**
 *
 * @param message
 */
void receive_nop(union CAN_logical_message *message) {
	printf("nop called\n");
}

/* which group is used */
struct message_sender *senders = NULL;
struct message_receiver *receivers = NULL;

static struct message_sender SENDERS_actuators[0x2F];
static struct message_receiver RECEIVERS_actuators[0x2F];
/* ======================== ACTUATORS ================== */
SENDER(06, actuators, ADDR_EMERGENCY, ADDR_TAIL_ENGINE) {
	SENDER_START();
	SEND(06, tail_engine_angle_X);
	SEND(06, tail_engine_angle_Y);
	SEND(06, tail_engine_fix);
	SEND(06, tail_engine_rate);
	FRAME_COUNT(06, 1); // first and last
	TEXT_SIZE_WORD(06, 14);
	SENDER_FINISH();
}

SENDER(2a, actuators, ADDR_EMERGENCY, ADDR_TAIL_ENGINE) {
	SENDER_START();
	SEND(2a, left_electromotor_rate);
	SEND(2a, left_electromotor_angle_X);
	SEND(2a, left_electromotor_angle_Y);
	SEND(2a, right_electromotor_rate);
	SEND(2a, right_electromotor_angle_X);
	SEND(2a, right_electromotor_angle_Y);

	TEXT_SIZE_WORD(2a, 24);
	FRAME_COUNT(2a, 3); // 25 bytes == 4 frames
	SENDER_FINISH();
}

RECEIVER(07, actuators) {
	RECEIVER_START();
	// TODO command confirmation
	RECEIVER_FINISH();
}

RECEIVER(08, actuators) {
	RECEIVER_START();
	RECEIVE(08, left_electromotor_angle_X);
	RECEIVE(08, left_electromotor_angle_Y);
	RECEIVE(08, left_electromotor_rate);
	RECEIVE(08, right_electromotor_angle_X);
	RECEIVE(08, right_electromotor_angle_Y);
	RECEIVE(08, right_electromotor_rate);
	RECEIVER_FINISH();
}

SENDER(0a, actuators, ADDR_EMERGENCY, ADDR_LEFT_MAIN_ENGINE_ROTATION) {
	SENDER_START();
	SEND(0a, left_main_engine_angle);
	SEND(0a, left_main_engine_fix);
	TEXT_SIZE_BYTE(0a, 6); // text size + FLOAT + byte + reserved = 7 bytes total
										   // but 6 is specified
	SENDER_FINISH();
}

RECEIVER(0b, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

RECEIVER(0c, actuators) {
	RECEIVER_START();
	RECEIVE(0c, left_main_engine_angle);
	RECEIVER_FINISH();
}

SENDER(0e, actuators, ADDR_EMERGENCY, ADDR_RIGHT_MAIN_ENGINE_ROTATION) {
	SENDER_START();
	SEND(0e, right_main_engine_angle);
	SEND(0e, right_main_engine_fix);
	TEXT_SIZE_BYTE(0e, 6);
	SENDER_FINISH();
}

RECEIVER(0f, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

RECEIVER(10, actuators) {
	RECEIVER_START();
	RECEIVE(10, right_main_engine_angle);
	RECEIVER_FINISH();
}

SENDER(14, actuators, ADDR_EMERGENCY, ADDR_LEFT_MAIN_ENGINE_CONTROL) {
	SENDER_START();
	SEND(14, left_main_engine_rate);
	SEND(14, left_main_engine_startstop);
	TEXT_SIZE_BYTE(14, 5);
	SENDER_FINISH();
}

RECEIVER(15, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

RECEIVER(16, actuators) {
	RECEIVER_START();
	RECEIVE(16, left_main_engine_rate);
	RECEIVER_FINISH();
}

SENDER(1a, actuators, ADDR_EMERGENCY, ADDR_RIGHT_MAIN_ENGINE_CONTROL) {
	SENDER_START();
	SEND(1a, right_main_engine_rate);
	SEND(1a, right_main_engine_startstop);
	TEXT_SIZE_BYTE(1a, 5);
	SENDER_FINISH();
}

RECEIVER(1b, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

RECEIVER(1c, actuators) {
	RECEIVER_START();
	RECEIVE(1c, right_main_engine_rate);
	RECEIVER_FINISH();
}

SENDER(1e, actuators, ADDR_EMERGENCY, ADDR_LEFT_BALLONET) {
	SENDER_START();
	SEND(1e, left_ballonet_control);
	SEND(1e, left_ballonet_pressure_bias);
	SEND(1e, left_ballonet_pressure_intake_threshold);
	SEND(1e, left_ballonet_pressure_set);
	SEND(1e, left_ballonet_pressure_suction_threshold);
	SEND(1e, left_ballonet_state);
	// FIXME единицы измерения * 100
	SEND_VAL(1e, left_ballonet_wind_speed, MODEL.left_ballonet_wind_speed * 100);

	FRAME_COUNT(1e, 2);
	TEXT_SIZE_WORD(1e, 17);
	SENDER_FINISH();
}

RECEIVER(1f, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

RECEIVER(20, actuators) {
	RECEIVER_START();
	RECEIVE(20, left_ballonet_differential_pressure_1);
	RECEIVE(20, left_ballonet_differential_pressure_2);
	RECEIVE(20, left_ballonet_state);
	RECEIVER_FINISH();
}

SENDER(22, actuators, ADDR_EMERGENCY, ADDR_RIGHT_BALLONET) {
	SENDER_START();
	FRAME_COUNT(22, 3);
	TEXT_SIZE_WORD(22, 17);

	SEND(22, right_ballonet_control);
	SEND(22, right_ballonet_pressure_bias);
	SEND(22, right_ballonet_pressure_intake_threshold);
	SEND(22, right_ballonet_pressure_set);
	SEND(22, right_ballonet_pressure_suction_threshold);
	SEND(22, right_ballonet_state);
	// FIXME единицы измерения * 100
	SEND_VAL(22, right_ballonet_wind_speed, MODEL.right_ballonet_wind_speed * 100);
	SENDER_FINISH();
}

RECEIVER(23, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

RECEIVER(24, actuators) {
	RECEIVER_START();
	RECEIVE(24, right_ballonet_differential_pressure_1);
	RECEIVE(24, right_ballonet_differential_pressure_2);
	RECEIVE(24, right_ballonet_state);
	RECEIVER_FINISH();
}

SENDER(26, actuators, ADDR_EMERGENCY, ADDR_POWER) {
	SENDER_START();
	SEND(26, AUTONOMOUS_POWER_DISTRIBUTION_MODE);
	SEND(26, K1_8);
	SEND(26, K9_16);
	SEND(26, K17_24);
	SEND(26, K25_32);
	SEND(26, airship_manual_power_control);
	TEXT_SIZE_BYTE(26, 8);
	SENDER_FINISH();
}

RECEIVER(27, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

RECEIVER(28, actuators) {
	RECEIVER_START();
	RECEIVER_FINISH();
}

/* =============== ACTUATORS RESERVED ============== */
SENDER(12, actuators, ADDR_EMERGENCY, ADDR_LEFT_MAIN_ENGINE_CONTROL) {
	SENDER_START();
	SEND(12, left_main_engine_fuel_air_ratio);
	SEND(12, left_main_engine_throttle);
	TEXT_SIZE_BYTE(12, 5);
	SENDER_FINISH();
}

SENDER(18, actuators, ADDR_EMERGENCY, ADDR_RIGHT_MAIN_ENGINE_CONTROL) {
	SENDER_START();
	SEND(18, right_main_engine_fuel_air_ratio);
	SEND(18, right_main_engine_throttle);
	TEXT_SIZE_BYTE(18, 5);
	SENDER_FINISH();
}

/**
 *
 */
static struct message_receiver RECEIVERS_emulator[0x2F];
/**
 *
 */
static struct message_sender SENDERS_emulator[0x2F];

/* ============================ EMULATOR ==================== */
RECEIVER(06, emulator) {
	RECEIVER_START();
	RECEIVE(06, tail_engine_angle_X);
	RECEIVE(06, tail_engine_angle_Y);
	RECEIVE(06, tail_engine_fix);
	RECEIVE(06, tail_engine_rate);
	QUEUE_MATLAB_NOTIFY(tail_engine_rate, "tail engine rate %hu");
	QUEUE_MATLAB_NOTIFY(tail_engine_angle_X, "tail engine anglex %f");
	QUEUE_MATLAB_NOTIFY(tail_engine_angle_Y, "tail engine angley %f");
	QUEUE_MATLAB_NOTIFY(tail_engine_fix == MAIN_ENGINE_FIX_FORCE, "tail engine fix %d");
	RECEIVER_FINISH();
	enqueue_MID(0x07);
}

SENDER(07, emulator, ADDR_TAIL_ENGINE, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(07, 0x03);
	SEND_VAL(07, text_size, 0x03);
	SEND_VAL(07, command_number, 0x06);
	SEND_VAL(07, MID_message_number, 0x07);
	SENDER_FINISH();
}

// FIXME temporary message
RECEIVER(2a, emulator) {
	RECEIVER_START();

	RECEIVE(2a, left_electromotor_rate);
	RECEIVE(2a, left_electromotor_angle_X);
	RECEIVE(2a, left_electromotor_angle_Y);

	QUEUE_MATLAB_NOTIFY(left_electromotor_rate, "left electromotor rate %hu");
	QUEUE_MATLAB_NOTIFY(left_electromotor_angle_X, "left electromotor anglex %f");
	QUEUE_MATLAB_NOTIFY(left_electromotor_angle_Y, "left electromotor angley %f");

	RECEIVE(2a, right_electromotor_rate);
	RECEIVE(2a, right_electromotor_angle_X);
	RECEIVE(2a, right_electromotor_angle_Y);

	QUEUE_MATLAB_NOTIFY(right_electromotor_rate, "right electromotor rate %hu");
	QUEUE_MATLAB_NOTIFY(right_electromotor_angle_X, "right electromotor anglex %f");
	QUEUE_MATLAB_NOTIFY(right_electromotor_angle_Y, "right electromotor angley %f");

	RECEIVER_FINISH();
	enqueue_MID(0x07);
}


SENDER(08, emulator, ADDR_TAIL_ENGINE, ADDR_EMERGENCY) {
	SENDER_START();
	/* values are substituted in packed_finished_emulator with value of tail engine */
	SEND(08, left_electromotor_rate);
	SEND(08, right_electromotor_rate);
	SEND(08, left_electromotor_angle_X);
	SEND(08, left_electromotor_angle_Y);
	SEND(08, right_electromotor_angle_X);
	SEND(08, right_electromotor_angle_Y);
	FRAME_COUNT(08, 4); // 3 == first, 2 middle and last, 3, 2, 1, 0
	TEXT_SIZE_WORD(08, 32); // 33 bytes total
	SENDER_FINISH();
}

RECEIVER(0a, emulator) {
	RECEIVER_START();
	RECEIVE(0a, left_main_engine_angle);
	RECEIVE(0a, left_main_engine_fix);
	QUEUE_MATLAB_NOTIFY(left_main_engine_angle, "left engine angle %f");
	QUEUE_MATLAB_NOTIFY(left_main_engine_fix  == MAIN_ENGINE_FIX_FORCE, "left engine fix %d");
	RECEIVER_FINISH();
	enqueue_MID(0x0b);
}

SENDER(0b, emulator, ADDR_LEFT_MAIN_ENGINE_ROTATION, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(0b, 0x03);
	SEND_VAL(0b, text_size, 0x03);
	SEND_VAL(0b, command_number, 0x0a);
	SEND_VAL(0b, MID_message_number, 0x0b);
	SENDER_FINISH();
}

SENDER(0c, emulator, ADDR_LEFT_MAIN_ENGINE_ROTATION, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(0c, 0x05);
	SEND(0c, left_main_engine_angle);
	SENDER_FINISH();
}

RECEIVER(0e, emulator) {
	RECEIVER_START();
	RECEIVE(0e, right_main_engine_angle);
	RECEIVE(0e, right_main_engine_fix);
	QUEUE_MATLAB_NOTIFY(right_main_engine_angle, "right engine angle %f");
	QUEUE_MATLAB_NOTIFY(right_main_engine_fix  == MAIN_ENGINE_FIX_FORCE, "right engine fix %d");
	RECEIVER_FINISH();
	enqueue_MID(0x0f);
}

SENDER(0f, emulator, ADDR_RIGHT_MAIN_ENGINE_ROTATION, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(0f, 0x03);
	SEND_VAL(0f, text_size, 0x03);
	SEND_VAL(0f, command_number, 0x0e);
	SEND_VAL(0f, MID_message_number, 0x0f);
	SENDER_FINISH();
}

SENDER(10, emulator, ADDR_RIGHT_MAIN_ENGINE_ROTATION, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(10, 0x05);
	SEND(10, right_main_engine_angle);
	SENDER_FINISH();
}

RECEIVER(14, emulator) {
	RECEIVER_START();
	RECEIVE(14, left_main_engine_rate);
	RECEIVE(14, left_main_engine_startstop);
	QUEUE_MATLAB_NOTIFY(left_main_engine_rate, "left engine rate %hu");
	/* FIXME startstop command to matlab */
	RECEIVER_FINISH();
	enqueue_MID(0x15);
}

SENDER(15, emulator, ADDR_LEFT_MAIN_ENGINE_CONTROL, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(15, 0x03);
	SEND_VAL(15, text_size, 0x03);
	SEND_VAL(15, command_number, 0x14);
	SEND_VAL(15, MID_message_number, 0x15);
	SENDER_FINISH();
}

SENDER(16, emulator, ADDR_LEFT_MAIN_ENGINE_CONTROL, ADDR_EMERGENCY) {
	SENDER_START();
	FRAME_COUNT(16, 4); // 5 frames
	TEXT_SIZE_WORD(16, 36); // 37 bytes total
	SEND(16, left_main_engine_rate);
	// FIXME a lot of data
	SENDER_FINISH();
}

RECEIVER(1a, emulator) {
	RECEIVER_START();
	RECEIVE(1a, right_main_engine_rate);
	RECEIVE(1a, right_main_engine_startstop);
	QUEUE_MATLAB_NOTIFY(right_main_engine_rate, "right engine rate %hu");
	/* FIXME startstop command to matlab */
	RECEIVER_FINISH();
	enqueue_MID(0x1b);
}

SENDER(1b, emulator, ADDR_RIGHT_MAIN_ENGINE_CONTROL, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(1b, 0x03);
	SEND_VAL(1b, text_size, 0x03);
	SEND_VAL(1b, command_number, 0x1a);
	SEND_VAL(1b, MID_message_number, 0x1b);
	SENDER_FINISH();
}

SENDER(1c, emulator, ADDR_RIGHT_MAIN_ENGINE_CONTROL, ADDR_EMERGENCY) {
	SENDER_START();
	FRAME_COUNT(1c, 4); // 5 frames
	TEXT_SIZE_WORD(16, 36); // 37 bytes total
	SEND(1c, right_main_engine_rate);
	SENDER_FINISH();
}

RECEIVER(1e, emulator) {
	RECEIVER_START();
	RECEIVE(1e, left_ballonet_control);
	RECEIVE(1e, left_ballonet_pressure_bias);
	RECEIVE(1e, left_ballonet_pressure_intake_threshold);
	RECEIVE(1e, left_ballonet_pressure_set);
	RECEIVE(1e, left_ballonet_pressure_suction_threshold);
	RECEIVE(1e, left_ballonet_state);
	/* FIXME more complex ballonet state */
	QUEUE_MATLAB_NOTIFY(left_ballonet_state.ballonet_fan_1 == COMMAND_BALLONET_FORCE_OPEN, "left ballonet fan1 %d");
	QUEUE_MATLAB_NOTIFY(left_ballonet_state.ballonet_fan_2 == COMMAND_BALLONET_FORCE_OPEN, "left ballonet fan2 %d");
	QUEUE_MATLAB_NOTIFY(left_ballonet_state.ballonet_valve_1 == COMMAND_BALLONET_FORCE_OPEN, "left ballonet valve %d");
	QUEUE_MATLAB_NOTIFY(left_ballonet_state.ballonet_valve_2 == COMMAND_BALLONET_FORCE_OPEN, "helium valve %d");

	// FIXME единицы измерения * 100
	RECEIVE(1e, left_ballonet_wind_speed);
	RECEIVER_FINISH();
	enqueue_MID(0x1f);
}

SENDER(1f, emulator, ADDR_LEFT_BALLONET, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(1f, 0x03);
	SEND_VAL(1f, text_size, 0x03);
	SEND_VAL(1f, command_number, 0x1e);
	SEND_VAL(1f, MID_message_number, 0x1f);
	SENDER_FINISH();
}

SENDER(20, emulator, ADDR_LEFT_BALLONET, ADDR_EMERGENCY) {
	SENDER_START();
	FRAME_COUNT(20, 1); // 2 frames
	TEXT_SIZE_WORD(20, 11); // 12 bytes total
	SEND(20, left_ballonet_differential_pressure_1);
	SEND(20, left_ballonet_differential_pressure_2);
	SEND(20, left_ballonet_state);
	SENDER_FINISH();
}

RECEIVER(22, emulator) {
	RECEIVER_START();
	RECEIVE(22, right_ballonet_control);
	RECEIVE(22, right_ballonet_pressure_bias);
	RECEIVE(22, right_ballonet_pressure_intake_threshold);
	RECEIVE(22, right_ballonet_pressure_set);
	RECEIVE(22, right_ballonet_pressure_suction_threshold);
	RECEIVE(22, right_ballonet_state);
	QUEUE_MATLAB_NOTIFY(right_ballonet_state.ballonet_fan_1 == COMMAND_BALLONET_FORCE_OPEN, "right ballonet fan1 %d");
	QUEUE_MATLAB_NOTIFY(right_ballonet_state.ballonet_fan_2 == COMMAND_BALLONET_FORCE_OPEN, "right ballonet fan2 %d");
	QUEUE_MATLAB_NOTIFY(right_ballonet_state.ballonet_valve_1 == COMMAND_BALLONET_FORCE_OPEN, "right ballonet valve %d");
	// FIXME единицы измерения * 100
	RECEIVE(22, right_ballonet_wind_speed);
	RECEIVER_FINISH();
	enqueue_MID(0x23);
}

SENDER(23, emulator, ADDR_RIGHT_BALLONET, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(23, 0x03);
	SEND_VAL(23, text_size, 0x03);
	SEND_VAL(23, command_number, 0x22);
	SEND_VAL(23, MID_message_number, 0x23);
	SENDER_FINISH();
}

SENDER(24, emulator, ADDR_RIGHT_BALLONET, ADDR_EMERGENCY) {
	SENDER_START();
	FRAME_COUNT(24, 1); // 2 frames total
	TEXT_SIZE_WORD(24, 11); // 12 bytes total
	SEND(24, right_ballonet_differential_pressure_1);
	SEND(24, right_ballonet_differential_pressure_2);
	SEND(24, right_ballonet_state);
	SENDER_FINISH();
}

RECEIVER(26, emulator) {
	RECEIVER_START();
	RECEIVE(26, AUTONOMOUS_POWER_DISTRIBUTION_MODE);
	RECEIVE(26, K1_8);
	RECEIVE(26, K9_16);
	RECEIVE(26, K17_24);
	RECEIVE(26, K25_32);
	RECEIVE(26, airship_manual_power_control);
	RECEIVER_FINISH();
}

SENDER(27, emulator, ADDR_POWER, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_BYTE(27, 0x03);
	SEND_VAL(27, text_size, 0x03);
	SEND_VAL(27, command_number, 0x26);
	SEND_VAL(27, MID_message_number, 0x27);
	SENDER_FINISH();
}

SENDER(28, emulator, ADDR_POWER, ADDR_EMERGENCY) {
	SENDER_START();
	TEXT_SIZE_WORD(28, 85); // 86 bytes total
	FRAME_COUNT(28, 10); // 11 frames total
	// FIXME
	SENDER_FINISH();
}


/* ================= EMULATOR RESERVED =================== */
RECEIVER(12, emulator) {
	RECEIVER_START();
	RECEIVE(12, left_main_engine_fuel_air_ratio);
	RECEIVE(12, left_main_engine_throttle);
	RECEIVER_FINISH();
}


RECEIVER(18, emulator) {
	RECEIVER_START();
	RECEIVE(18, right_main_engine_fuel_air_ratio);
	RECEIVE(18, right_main_engine_throttle);
	RECEIVER_FINISH();
}

/**
 * handle special cases
 * @param MID
 * @param id
 */
void handle_CAN_id_exceptions(int MID, union CAN_id *id) {

}

/*  */
/**
 *
 * @todo	 retry indefinetely
 * @param CAN_id
 * @param sendbuf
 * @param dlc
 * @return
 */
int CAN_send(union CAN_id *CAN_id, uint8_t *sendbuf, int dlc) {
	int ret = 0;
	int first_time = 1;

	do {
		if (first_time) {
			printf("sending bytes ");
			first_time = 0;
		} else {
			printf("retrying packet enqueue ");
		}
		print_hex_bytes(sendbuf, dlc);
		printf(" to id ");
		print_hex_bytes((unsigned char *) &CAN_id->plain_id, 4);
		printf("\n");
		ret = checkCanLibCall("canWrite", canWrite(h, CAN_id->plain_id, sendbuf, dlc, canMSG_EXT));
	} while (ret != canOK);

	return ret;
}

/**
 * wait 1 second for all queued CAN messages to go on the bus
 */
void CAN_flush() {
	int done = 0;
	while (!done) {
		if (checkCanLibCall("canWriteSync", canWriteSync(h, 100)) != canOK) {
			fprintf(stderr, "Sync failed!\n");
		} else
			done = 1;
	}
}

/**
 * construct a message of given type, taking data from globally available
 * model and send it through CAN interface
 *
 * Not thread-safe yet.
 * @param MID
 */
void send_message(int MID) {
	uint16_t text_size = 0;
	uint16_t sent_text_size = 0;
	uint8_t frame_count = 0;

	int frame_number = 0;
//	int result = 0;

	/* FIXME check operation results */

	union CAN_logical_message message;
	union CAN_id CAN_id;
	union CAN_message_start_combined message_start;

	message_start.start.MID = MID;
	message_start.start.NUM = frame_number;

	memset(message.data, 0, sizeof(message.data));
	CAN_id.plain_id = 0;
	memset(message_start.bytes, 0, sizeof(message_start.bytes));

	/* generate binary data and get frame_count/text_size of this data*/
	senders[MID].send(&message, &frame_count, &text_size);
	printf("transport text size %hu\n", text_size);

	/* fill persistent ID parts */
	CAN_id.actuators_id.destination = senders[MID].destination;
	CAN_id.actuators_id.source = senders[MID].source;
	CAN_id.actuators_id.priority = senders[MID].priority;

	/* prepare message start header */
	message_start.start.MID = MID;
	message_start.start.NUM = 0;
	message_start.start.MSX = senders[MID].MSX;

	/* apply non-table-defined ID modifications */
	handle_CAN_id_exceptions(MID, &CAN_id);

	/* split messages if necessary */
	if (frame_count != 0) {
		// multiple messages, 2 at least

		// 1. send first message start + text
		CAN_id.actuators_id.first_of_many_marker = MARK_FIRST_MANY_MESSAGE_START;

		CAN_send(&CAN_id, message_start.bytes, sizeof(message_start.bytes));

		CAN_id.actuators_id.first_of_many_marker = MARK_FIRST_MANY_MESSAGE_TEXT;

		CAN_send(&CAN_id, message.data + sent_text_size, 8);

		sent_text_size += 8;
		text_size -= 8;

		CAN_id.actuators_id.first_of_many_marker = MARK_FIRST_MANY_MESSAGE_NONE;

		for (frame_number = 1; frame_number < frame_count; frame_number++) {
			// 2*. send middle messages start + text
			message_start.start.NUM = frame_number;

			CAN_id.actuators_id.middle_of_many_marker = MARK_MIDDLE_MANY_MESSAGE_START;

			CAN_send(&CAN_id, message_start.bytes, sizeof(message_start.bytes));

			CAN_id.actuators_id.middle_of_many_marker = MARK_MIDDLE_MANY_MESSAGE_TEXT;

			CAN_send(&CAN_id, message.data + sent_text_size, 8);

			sent_text_size += 8;
			text_size -= 8;

			CAN_id.actuators_id.middle_of_many_marker = MARK_MIDDLE_MANY_MESSAGE_NONE;
		}
		// 3. send last message start + text

		message_start.start.NUM = frame_count;

		CAN_id.actuators_id.last_of_many_marker = MARK_LAST_MANY_MESSAGE_START;

		CAN_send(&CAN_id, message_start.bytes, sizeof(message_start.bytes));

		CAN_id.actuators_id.last_of_many_marker = MARK_LAST_MANY_MESSAGE_TEXT;

		printf("text size %hu\n", text_size);
		assert(text_size <= 8);
		CAN_send(&CAN_id, message.data + sent_text_size, text_size);

		CAN_id.actuators_id.last_of_many_marker = MARK_LAST_MANY_MESSAGE_NONE;

	} else {
		// send single message start + text
		message_start.start.NUM = frame_count;

		CAN_id.actuators_id.single_message_marker = MARK_SINGLE_MESSAGE_START;

		CAN_send(&CAN_id, message_start.bytes, sizeof(message_start.bytes));

		CAN_id.actuators_id.single_message_marker = MARK_SINGLE_MESSAGE_TEXT;

		assert(text_size <= 8);
		assert(message.data[0] != 0);
		CAN_send(&CAN_id, &message.data[0], text_size);

		CAN_id.actuators_id.single_message_marker = MARK_SINGLE_MESSAGE_NONE;

	}

	CAN_flush();
}

/**
 * Run corresponding receive function upon given MID and message body
 * @param MID
 * @param message
 */
void receive_message(int MID, union CAN_logical_message *message) {
	printf("receiving MID %02X\n", MID);
	receivers[MID].receive(message);
}

/**
 * Actuators and emulator should react differently to messages
 * @param message
 */
static void (*packet_finished_callback)(struct CAN_message *message);

/**
 * Respond to complete CAN message as actuators program
 * @param message
 */
void packet_finished_actuators(struct CAN_message *message) {
	printf("transport level message finished\n");
	print_hex_bytes(THIS_RECEIVER.message.data, THIS_RECEIVER.text_size);
	printf("\n");

	receive_message(THIS_RECEIVER.MID, &THIS_RECEIVER.message);

	switch (THIS_RECEIVER.MID) {
	case 0x00:
		break;
	case 0x01:
		break;
	case 0x02:
		break;
	case 0x03:
		break;
	case 0x04:
		break;
	case 0x05:
		break;
	case 0x06:
		break;
	case 0x07:
		break;
	case 0x08:
		break;
	case 0x09:
		break;
	case 0x0A:
		break;
	case 0x0B:
		break;
	case 0x0C:
		break;
	case 0x0D:
		break;
	case 0x0E:
		break;
	case 0x0F:
		break;
	case 0x10:
		break;
	case 0x11:
		break;
	case 0x12:
		break;
	case 0x13:
		break;
	case 0x14:
		break;
	case 0x15:
		break;
	case 0x16:
		break;
	case 0x17:
		break;
	case 0x18:
		break;
	case 0x19:
		break;
	case 0x1A:
		break;
	case 0x1B:
		break;
	case 0x1C:
		break;
	case 0x1D:
		break;
	case 0x1E:
		break;
	case 0x1F:
		break;
	case 0x20:
		break;
	case 0x21:
		break;
	case 0x22:
		break;
	case 0x23:
		break;
	case 0x24:
		break;
	case 0x25:
		break;
	case 0x26:
		break;
	case 0x27:
		break;
	case 0x28:
		break;
	case 0x29:
		break;
	case 0x2A:
		break;
	case 0x2B:
		break;
	case 0x2C:
		break;
	case 0x2D:
		break;
	case 0x2E:
		break;
	case 0x2F:
		break;
	default:
		fprintf(stderr, "unknown message ID encountered!!!!!\n");
		break;
	}
}

/**
 * Respond to complete CAN message as emulator
 * @param message
 */
void packet_finished_emulator(struct CAN_message *message) {
	printf("transport level message finished\n");
	print_hex_bytes(THIS_RECEIVER.message.data, THIS_RECEIVER.text_size);

	printf("\n");

	receive_message(THIS_RECEIVER.MID, &THIS_RECEIVER.message);

	switch (THIS_RECEIVER.MID) {
	case 0x00:
		break;
	case 0x01:
		break;
	case 0x02:
		break;
	case 0x03:
		break;
	case 0x04:
		break;
	case 0x05:
		break;
	case 0x06:
		break;
	case 0x07:
		break;
	case 0x08:
		break;
	case 0x09:
		break;
	case 0x0A:
		break;
	case 0x0B:
		break;
	case 0x0C:
		break;
	case 0x0D:
		break;
	case 0x0E:
		break;
	case 0x0F:
		break;
	case 0x10:
		break;
	case 0x11:
		break;
	case 0x12:
		break;
	case 0x13:
		break;
	case 0x14:
		break;
	case 0x15:
		break;
	case 0x16:
		break;
	case 0x17:
		break;
	case 0x18:
		break;
	case 0x19:
		break;
	case 0x1A:
		break;
	case 0x1B:
		break;
	case 0x1C:
		break;
	case 0x1D:
		break;
	case 0x1E:
		break;
	case 0x1F:
		break;
	case 0x20:
		break;
	case 0x21:
		break;
	case 0x22:
		break;
	case 0x23:
		break;
	case 0x24:
		break;
	case 0x25:
		break;
	case 0x26:
		break;
	case 0x27:
		break;
	case 0x28:
		break;
	case 0x29:
		break;
	case 0x2A:
		break;
	case 0x2B:
		break;
	case 0x2C:
		break;
	case 0x2D:
		break;
	case 0x2E:
		break;
	case 0x2F:
		break;
	default:
		fprintf(stderr, "unknown message ID encountered!!!!!\n");
		break;
	}
}


static inline void print_id(FILE *dest, union CAN_id id) {
	print_hex_bytes_generic(dest, (uint8_t *) &id.plain_id, sizeof(id.plain_id));
}

#define ERROR_HERE() do {\
	fprintf(stderr, "error in line %d\n", __LINE__);\
} while(0)

/**
 * messages are START/TEXT pairs, MID is known only for START, so
 * upon receiving TEXT message we look for expected id to group
 * messages together. I hope it's possible to differ them
 * @param id
 * @return
 */
static union CAN_id expect_text_marker(union CAN_id id) {
	union CAN_id result = id;
	char result_set = 0;

	switch (id.actuators_id.single_message_marker) {
	case MARK_SINGLE_MESSAGE_START:
		result.actuators_id.single_message_marker = MARK_SINGLE_MESSAGE_TEXT;
		result_set = 1;
		break;
	case MARK_SINGLE_MESSAGE_NONE:
		break;
	default:
		if (result_set) {
			ERROR_HERE();
			goto invalid_id;
		}
		break;
	}

	switch (id.actuators_id.first_of_many_marker) {
	case MARK_FIRST_MANY_MESSAGE_START:
		if (!result_set) {
			result.actuators_id.first_of_many_marker = MARK_FIRST_MANY_MESSAGE_TEXT;
			result_set = 1;
		} else {
			ERROR_HERE();
			goto invalid_id;
		}
		break;
	case MARK_FIRST_MANY_MESSAGE_NONE:
		break;
	default:
		if (result_set) {
			ERROR_HERE();
			goto invalid_id;
		}
		break;
	}

	switch (id.actuators_id.middle_of_many_marker) {
	case MARK_MIDDLE_MANY_MESSAGE_START:
		if (!result_set) {
			result.actuators_id.middle_of_many_marker = MARK_MIDDLE_MANY_MESSAGE_TEXT;
			result_set = 1;
		} else {
			ERROR_HERE();
			goto invalid_id;
		}
		break;
	case MARK_MIDDLE_MANY_MESSAGE_NONE:
		break;
	default:
		if (result_set) {
			ERROR_HERE();
			goto invalid_id;
		}
		break;
	}

	switch (id.actuators_id.last_of_many_marker) {
	case MARK_LAST_MANY_MESSAGE_START:
		if (!result_set) {
			result.actuators_id.last_of_many_marker = MARK_LAST_MANY_MESSAGE_TEXT;
			result_set = 1;
		} else {
			ERROR_HERE();
			goto invalid_id;
		}
		break;
	case MARK_LAST_MANY_MESSAGE_NONE:
		break;
	default:
		if (result_set) {
			ERROR_HERE();
			goto invalid_id;
		}
		break;
	}

	if (!result_set) {
		ERROR_HERE();
		fprintf(stderr, "no markers found\n");
		goto invalid_id;
	}

	return result;

invalid_id:
	fprintf(stderr, "invalid ID detected: ");
	print_id(stderr, id);
	fprintf(stderr, "\n");

	return result;
}

/**
 *
 * @param id
 * @return
 */
int get_receiver_index_by_id(union CAN_id id) {
	int idx = -1;

	int i = 0;

	for (i = 0; i < CAN_MESSAGE_MAX_ID; i++) {
		if (id.plain_id == RECEIVER.receiver[i].next_id.plain_id) {
			idx = i;
			break;
		}
	}

	if (idx == -1) {
		fprintf(stderr, "couldn't find id ");
		print_id(stderr, id);
		fprintf(stderr, "\n");
	}

	return idx;
}

/**
 *
 * @param message
 */
void process_transport_level(struct CAN_message *message) {
	switch (get_CAN_message_type(message)) {
	case CAN_message_first_of_many_start:
		memset((char *) &THIS_RECEIVER, 0, sizeof(THIS_RECEIVER));

		THIS_RECEIVER.text_size = 0;
		THIS_RECEIVER.frame_count = 0;
		THIS_RECEIVER.read_text_size = 0;

		/* no break */
	case CAN_message_single_start:
	case CAN_message_middle_of_many_start:
	case CAN_message_last_of_many_start:

		THIS_RECEIVER.MID =  message->message_start.MID;
		THIS_RECEIVER.MSX = message->message_start.MSX;
		// NUM = message->message_start.NUM;

		/* text message is received by id */
		THIS_RECEIVER.next_id = expect_text_marker(message->id);

		break;
	case CAN_message_single_text:
		/* text size == total text size aka dlc in this case */
		THIS_RECEIVER.text_size = message->data.single.text_size + 1;
		assert(THIS_RECEIVER.text_size <= 8);

		memcpy(THIS_RECEIVER.message.data, message->data.raw, THIS_RECEIVER.text_size);

		THIS_RECEIVER.read_text_size += THIS_RECEIVER.text_size;
		THIS_RECEIVER.next_id.plain_id = 0;

		packet_finished_callback(message);
		break;
	case CAN_message_first_of_many_text:

		THIS_RECEIVER.frame_count = message->data.first.frame_count;
		THIS_RECEIVER.text_size =
				message->data.first.text_size_high  * 256 +
				message->data.first.text_size_low;

		printf("TEXT SIZE IS %hd\n", THIS_RECEIVER.text_size);

		memcpy(THIS_RECEIVER.message.data, message->data.raw, message->dlc);

		THIS_RECEIVER.read_text_size += message->dlc;
		THIS_RECEIVER.next_id.plain_id = 0;

		break;
	case CAN_message_middle_of_many_text:

		memcpy(THIS_RECEIVER.message.data + THIS_RECEIVER.read_text_size,
				message->data.raw, message->dlc);

		THIS_RECEIVER.read_text_size += message->dlc;
		THIS_RECEIVER.next_id.plain_id = 0;

		break;

	case CAN_message_last_of_many_text:
		memcpy(THIS_RECEIVER.message.data + THIS_RECEIVER.read_text_size,
				message->data.raw, message->dlc);

		THIS_RECEIVER.read_text_size += message->dlc;
		THIS_RECEIVER.next_id.plain_id = 0;

		/* 1 is for FRAME_COUNT byte */
		if (THIS_RECEIVER.read_text_size != THIS_RECEIVER.text_size + FRAME_COUNT_BYTE_BIAS) {
			fprintf(stderr, "read text size %hd != given text size %hd + 1	\n",
					THIS_RECEIVER.read_text_size, THIS_RECEIVER.text_size);
		} else {
			packet_finished_callback(message);
		}

		break;
	case CAN_message_unknown:
	default:
		fprintf(stderr, "unknown CAN message type\n");
		break;
	}
}

/**
 *
 * @param message
 */
void print_CAN_message_debug(struct CAN_message *message) {
	if (CAN_MESSAGE_IS_SINGLE(message)) {
		printf("single\n");
	} else if (CAN_MESSAGE_IS_FIRST_OF_MANY(message)) {
		printf("first of many\n");
	} else if (CAN_MESSAGE_IS_MIDDLE_OF_MANY(message)) {
		printf("middle of many\n");
	} else if (CAN_MESSAGE_IS_LAST_OF_MANY(message)) {
		printf("last of many\n");
	} else {
		printf("ERROR: unknown kind of CAN message\n");
	}

	if (CAN_MESSAGE_IS_START(message)) {
		printf("is start\n");
		printf("NUM: %02hhX MID %02hhX MSX %02hhX\n",
				message->message_start.NUM,
				message->message_start.MID,
				message->message_start.MSX);
	} else if (CAN_MESSAGE_IS_TEXT(message)) {
		printf("is text\n");
	}

	pretty_print_message(message);

	printf("========================================\n");
	printf("========================================\n");
	printf("========================================\n");
};

/**
 *
 * @param id
 * @param msg
 * @param dlc
 * @param flag
 * @param message
 * @return
 */
char process_raw_CAN_message(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag, struct CAN_message *message)
{
	char result = 1;
	message->id.plain_id = id;
	message->dlc = dlc;
	static int last_MID = -1;

	memcpy(message->data.raw, msg, message->dlc);

	if (CAN_MESSAGE_IS_START(message)) {
		message->message_start.NUM = msg[0] & 31; /* trim reserved bits */
		message->message_start.MID = msg[1];
		message->message_start.MSX = msg[2] & 15;

	} else {
		int idx = -1;
		idx = get_receiver_index_by_id(message->id);
		if (idx == -1) {
			fprintf(stderr, "unexpected TEXT message with id: ");
			print_id(stderr, message->id);
			fprintf(stderr, "\n expected probably: ");
			print_id(stderr, RECEIVER.receiver[last_MID].next_id);
			fprintf(stderr, "\n");
			result = 0;
		} else {
			fprintf(stderr, "MID found for TEXT message is %02X\n", idx);
		}
		message->message_start.MID = RECEIVER.receiver[idx].MID;
		message->message_start.MSX = RECEIVER.receiver[idx].MSX;
		//message->message_start.NUM = RECEIVER.receiver[idx].NUM;
	}

	message->MID = message->message_start.MID;
	last_MID = message->MID;
	return result;
};

/**
 *
 * @param id
 * @param msg
 * @param dlc
 * @param flag
 */
void print_raw_CAN_message(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag)
{
	printf("got ID %s", MAKE_YELLOW);
	print_hex_bytes((unsigned char *) &id, 4);
	printf("%s DLC ", MAKE_BLUE);
	print_hex_bytes((unsigned char *) &dlc, 1);
	printf("%s DATA ", MAKE_GREEN);
	print_hex_bytes((unsigned char *) &msg[0], dlc);
	printf("%s\n", RESET_COLOR);
}

/**
 *
 * @param id
 * @param msg
 * @param dlc
 * @param flag
 */
void handle_CAN_message(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag)
{
	static struct CAN_message my_message;

	print_raw_CAN_message(id, msg, dlc, flag);

	if (process_raw_CAN_message(id, msg, dlc, flag, &my_message)) {
		print_CAN_message_debug(&my_message);
		process_transport_level(&my_message);
	}
}

void enqueue_MID(int MID) {
	SEND_QUEUE_LOCK();
	if (senders && !senders[MID].turned_off)
		send_queue_MID.push(MID);
	SEND_QUEUE_UNLOCK();
}

void *can_main(void *data)
{
	int ret = canOK;
	long id = 27;
	struct can_settings *can_settings;

	unsigned char msg[8];
	unsigned int dlc;
	unsigned int flag;
	unsigned long time;

	errno = 0;
	can_settings = (struct can_settings *) data;
	printf("Reading messages on channel %d\n", can_settings->channel);

	switch (can_settings->mode) {
	case 'a':
		packet_finished_callback = packet_finished_actuators;
		senders = SENDERS_actuators;
		receivers = RECEIVERS_actuators;
		break;
	case 'e':
		packet_finished_callback = packet_finished_emulator;
		senders = SENDERS_emulator;
		receivers = RECEIVERS_emulator;
		break;
	default:
		fprintf(stderr, "unknown mode %c\n", can_settings->mode);
		exit(-1);
		break;
	}

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	pthread_mutex_init(&MODEL_lock, &attr);
	pthread_mutex_init(&REPORTED_DATA_lock, &attr);
	pthread_mutex_init(&send_queue_lock, &attr);

	/* Open channels, parameters and go on bus */
	h = canOpenChannel(can_settings->channel,
			canOPEN_ACCEPT_VIRTUAL | canWANT_EXCLUSIVE | canWANT_EXTENDED);
	if (h < 0) {
		printf("canOpenChannel %d failed\n", can_settings->channel);
		goto out;
	}
	checkCanLibCall("parameters", canSetBusParams(h, can_settings->bitrate, 4, 3, 1, 1, 0));
	canSetBusOutputControl(h, canDRIVER_NORMAL);
	canBusOn(h);

	/* this part is for SIGINT to interrupt kvaser can call so this thread could exit gracefully */
	struct sigaction sa;
	sigset_t set;

	sigaddset(&set, SIGINT);
	pthread_sigmask(SIG_UNBLOCK, &set, NULL);
	sa.sa_flags = 0;
	sigemptyset(&sa.sa_mask);
	sa.sa_handler = sighand;
	if (sigaction(SIGINT, &sa, NULL) == -1) {
		perror("sigaction");
		pthread_exit(0);
	}

	while (!willExit) {
		while (ret == canOK) {
			switch (can_settings->mode) {
			case 'a':
			case 'e':
				/* wait 20 ms for messages */
                ret = canReadWait(h, &id, &msg, &dlc, &flag, &time, 200);
				switch (ret){
				case 0:
					if (flag & canMSG_ERROR_FRAME) {
						err++;
					} else {
						if (flag & canMSG_STD)
							standard++;
						if (flag & canMSG_EXT) { // our case
							handle_CAN_message(id, msg, dlc, flag);
							ext++;
						}
						if (flag & canMSG_RTR)
							rtr++;
						if (flag & canMSGERR_OVERRUN)
							over++;
					}
					i++;
					break;
				case canERR_NOMSG:
					ret = canOK;
					break;
				default:
					perror("canReadBlock error");
					break;
				};
				break;

			case 's':
				/* sender part */
				MODEL.tail_engine_angle_X = 5.0f;
				MODEL.tail_engine_angle_Y = 6.0f;
				MODEL.tail_engine_rate = 5500;
				enqueue_MID(0x06);
				ret = -1;
				sleep(1);
			break;
			default:
				ret = ~canOK;
				break;
			}

			/* drain send queue while there's nothing to read */
			while (!send_queue_MID.empty()) {
				SEND_QUEUE_LOCK();
				send_message(send_queue_MID.front());
				send_queue_MID.pop();
				SEND_QUEUE_UNLOCK();
			}
		}
		willExit = 1;
	}

	printf("Ready\n");
	sighand(SIGALRM);

out:
	return NULL;
}

#if 0
int main(int argc, char **argv) {

	int result = 0;

	if (argc != 3) {
		fprintf(stderr, "usage: ./transport channel_number mode\n");
		exit(1);
	}

	struct can_settings settings;
	settings.bitrate = BAUD_1M;
	settings.channel = atoi(argv[1]);
	settings.mode = argv[2][0];

	/* we want SIGINT to be passed to can thread */
	sigset_t set;
	sigaddset(&set, SIGINT);
	result = pthread_sigmask(SIG_BLOCK, &set, NULL);
	if (result) {
		perror("pthread_sigmask");
		exit(1);
	}

	if ((result = pthread_create(&can_thread, NULL, can_main, &settings))) {
		fprintf(stderr, "could'nt create can thread: %s %d\n", strerror(result), result);
		exit(1);
	}


	pthread_join(can_thread, NULL);
	return 0;
}
#endif
