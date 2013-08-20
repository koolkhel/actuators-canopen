#include <stdio.h>
#include <canfestival.h>
#include <data.h>
#include <sys/select.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>

#include "./actuators.h"
#include "./model.h"
#include "./canopen-util.h"
#include "./canopen-data.h"

#include <assert.h>

static char busname[] = "0";
static char baudrate[] = "500K";

s_BOARD Board = { "", "" };
FILE *error_log = NULL;

/**************************** INIT ********************************************/
void Init(CO_Data *d, UNS32 id) {
	if (Board.baudrate) {
		/* Init node state*/
		setState(&actuators_Data, Initialisation);
	}
}

/***************************  CLEANUP  *****************************************/
void Exit(CO_Data* d, UNS32 nodeid) {
	if (strcmp(Board.baudrate, "none")) {
		/* Stop master */
		setState(&actuators_Data, Stopped);
	}
}

int callback_no = 0;

struct actuators_model MODEL;
extern pthread_mutex_t MODEL_lock;

struct Indigo_OD_Callback callbacks[200];

#define RECEIVE(VALUE, NAME, SCALE) do {\
			MODEL.NAME = VALUE; \
			fprintf(stderr, "receiving %s\n", # NAME);\
		} while(0)

#define RECEIVE_ARRAY(VALUE, NAME) do {\
			memcpy(&MODEL.NAME,&VALUE,sizeof(VALUE)); \
			fprintf(stderr, "receiving %s\n", # NAME);\
		} while(0)


#define DECLARE_M(INDEX, SUBINDEX) \
	if (size != sizeof(struct data_## INDEX ## _## SUBINDEX)) {\
		fprintf(error_log, "%ld size mismatch: %d against reference %d\n", time(NULL), size, sizeof(struct data_## INDEX ## _## SUBINDEX));\
		return;\
	};\
	assert(size == sizeof(struct data_## INDEX ## _## SUBINDEX));\
	struct data_## INDEX ##_## SUBINDEX *M = (struct data_ ## INDEX ##_## SUBINDEX *) data

#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

#define RECEIVE_DEBUG(VALUE, NAME, SCALE, FORMAT) do {\
	MODEL.NAME = VALUE; \
	fprintf(stdout,"receiving " STRINGIFY(NAME) ": " FORMAT "", MODEL.NAME);\
	fprintf(stdout, " with raw bytes : "); \
	int offset = offsetof(typeof(M), NAME);\
	print_hex_bytes(((unsigned char *)&M + offset, sizeof(M.NAME));\
	fprintf(stdout, "\n");\
} while (0)

// EMERGENCY
CALLBACK(0x4002, 0x10) {

}

CALLBACK(0x4002, 0x20) {
	// failure_code
	DECLARE_M(4002, 20);

	RECEIVE_ARRAY(M->failure_code, failure_code);
}

CALLBACK(0x4002, 0x21) {
	// communications_state
	DECLARE_M(4002, 21);

	RECEIVE_ARRAY(M->link_status, link_status);
}

CALLBACK(0x4002, 0x22) {
	DECLARE_M(4002, 22);
	// power_system_state
	RECEIVE_ARRAY(M->power_status, power_status);
}

CALLBACK(0x4002, 0x23) {
	// left right motors
	DECLARE_M(4002, 23);

	RECEIVE_ARRAY(M->main_engine_status, main_engine_status);
}

CALLBACK(0x4002, 0x24) {
	// rotation state
	DECLARE_M(4002, 24);

	RECEIVE_ARRAY(M->rotation_status, rotation_status);
}

CALLBACK(0x4002, 0x25) {
	// misc state
	DECLARE_M(4002, 25);

	RECEIVE_ARRAY(M->misc_status, misc_status);
}

// ENVIRONMENT
CALLBACK(0x4003, 0x10) {
}

CALLBACK(0x4003, 0x20) {
	// TOdo many variables
	DECLARE_M(4003, 20);
}

// PAYLOAD_BLOCK
CALLBACK(0x4004, 0x10) {

}

CALLBACK(0x4004, 0x20) {
	DECLARE_M(4004, 20);
}

// POWER
CALLBACK(0x4005, 0x10) {

}

CALLBACK(0x4005, 0x20) {

}

CALLBACK(0x4005, 0x21) {
	DECLARE_M(4005, 21);

	RECEIVE_ARRAY(M->electromotor_relay_status, electromotor_relay_status);
}

// TAIL_ENGINE
CALLBACK(0x4007, 0x10) {

}


CALLBACK(0x4007, 0x20) {
	DECLARE_M(4007, 20);

	RECEIVE(M->left_electromotor_angle_X, left_electromotor_angle_X, 100.0);
	RECEIVE(M->left_electromotor_angle_Y, left_electromotor_angle_Y, 100.0);
	RECEIVE(M->left_electromotor_rate, left_electromotor_rate, 1);

	RECEIVE(M->right_electromotor_angle_X, right_electromotor_angle_X, 100.0);
	RECEIVE(M->right_electromotor_angle_Y, right_electromotor_angle_Y, 100.0);
	RECEIVE(M->right_electromotor_rate, right_electromotor_rate, 1);
}

CALLBACK(0x4007, 0x21) {

}

// LEFT MOTOR
CALLBACK(0x4008, 0x10) {

}

CALLBACK(0x4008, 0x20) {
	DECLARE_M(4008, 20);

	RECEIVE(M->left_electromotor_rate, left_electromotor_rate, 1);
}

// RIGHT MOTOR
CALLBACK(0x4009, 0x10) {
}

CALLBACK(0x4009, 0x20) {
	DECLARE_M(4009, 20);

	RECEIVE(M->right_electromotor_rate, right_electromotor_rate, 1);
}

// LEFT_MOTOR_ROTATE
CALLBACK(0x400a, 0x10) {

}

CALLBACK(0x400a, 0x20) {
	DECLARE_M(400a, 20);

	RECEIVE(M->left_main_engine_rotation_angle, left_main_engine_rotation_angle, 100.0);
}

CALLBACK(0x400a, 0x21) {

}

// RIGHT_MOTOR_ROTATE
CALLBACK(0x400b, 0x20) {
	DECLARE_M(400b, 20);

	RECEIVE(M->right_main_engine_rotation_angle, right_main_engine_rotation_angle, 100.0);
}

CALLBACK(0x400b, 0x21) {
	double voltage = (data[0] + data[1] * 256) / 10.0;

	printf("left motor rotation voltage %lf\n", voltage);
}



CALLBACK(0x400c, 0x10) {

}

CALLBACK(0x400c, 0x20) {
	// остальное не читается, только задаётся
	// 0x200, 0x300, 0x400, 0x500 + 12

	printf("SIZE IS %d\n", size);
	DECLARE_M(400c, 20);

	RECEIVE(M->left_ballonet_pressure_1, left_ballonet_pressure_1, 1);
	RECEIVE(M->left_ballonet_pressure_2, left_ballonet_pressure_2, 1);
	RECEIVE(M->left_ballonet_control, left_ballonet_control, 1);
}

CALLBACK(0x400d, 0x10) {

}

CALLBACK(0x400d, 0x20) {
	// остальное не читается, только задаётся
	// 0x200, 0x300, 0x400, 0x500 + 13
	// right_ballonet_state.ballonet_fan_1
	// right_ballonet_state.ballonet_fan_2
	// right_ballonet_state.ballonet_valve_1

	// right_ballonet_differential_pressure_1
	// right_ballonet_differential_pressure_2
	DECLARE_M(400d, 20);

	RECEIVE(M->right_ballonet_pressure_1, right_ballonet_pressure_1, 1);
	RECEIVE(M->right_ballonet_pressure_2, right_ballonet_pressure_2, 1);
	RECEIVE(M->right_ballonet_control, right_ballonet_control, 1);
}

int CallSDOCallback(UNS16 index, UNS8 subindex, UNS8 *data, UNS32 size) {
	UNS16 i = 0;
	int found = 0;
	for (i = 0; i < sizeof(callbacks) / sizeof(callbacks[0]); i++) {
		if ((callbacks[i].index == index)
				&& (callbacks[i].subindex == subindex)) {
			found = 1;
			(callbacks[i].callback_fn)(data, size);
		}
	}

	return found;
}

/* Callback function that check the read SDO demand */
void CheckReadSDO(CO_Data *d, UNS8 nodeid) {
	UNS32 abortCode;
	UNS32 size = 64;
	UNS8 data[20] = { 0 };
	UNS16 index = 0;
	UNS8 subindex = 0;
	UNS8 err = 0;

	err = _getIndexSubindex(d, nodeid, &index, &subindex);
	if (err) {
		fprintf(error_log, "%ld couldn't find getIndexSubindex for nodeid 0x%hhx index 0x%hx\n", nodeid, index, time(NULL));
		goto out;
	}

	if (getReadResultNetworkDict(d, nodeid, &data, &size,
			&abortCode) != SDO_FINISHED)
		printf(
				"\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n",
				nodeid, abortCode);
	else {
		printf("\nResult : ");
		print_hex((char *) data, size);
		printf("\n");
		CallSDOCallback(index, subindex, data, size);
	}

out:
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(d, nodeid, SDO_CLIENT);
}

/* regularily poll these remote OD entries */
struct pollable_OD_entry {
	UNS8 nodeId;
	UNS16 index;
	UNS8 subindex;
	UNS8 datatype;
};

struct pollable_OD_entry pollable_entries[] = {
	//	{0x4005, 0x20},
		{0x7, 0x4007, 0x20, octet_string},
		{0xa, 0x400a, 0x20, octet_string},
		{0xb, 0x400b, 0x20, octet_string},
		{0xc, 0x400c, 0x20, octet_string},
		{0xd, 0x400d, 0x20, octet_string}
};

std::queue<int> send_queue_COB;
pthread_mutex_t send_queue_COB_lock;

pthread_mutex_t MODEL_lock;
pthread_mutex_t REPORTED_DATA_lock;

/* ROS services use this for command mappings */
void enqueue_COB(int COB) {
	SEND_QUEUE_LOCK();
	send_queue_COB.push(COB);
	SEND_QUEUE_UNLOCK();
}


void CANopen_startup(void) {
	Board.busname = busname;
	Board.baudrate = baudrate;

	LoadCanDriver("/usr/local/lib/libcanfestival_can_kvaser.so");

	setNodeId(&actuators_Data, 0x1);

	/* Init stack timer */
	TimerInit();

	canOpen(&Board, &actuators_Data);

	/* Start Timer thread */
	StartTimerLoop(&Init);

	setState(&actuators_Data, Pre_operational);

	my_sleep(1);

	setState(&actuators_Data, Operational);
}

void CANopen_shutdown(void) {
	// Stop timer thread
	StopTimerLoop(&Exit);

	canClose(&actuators_Data);

	TimerCleanup();
}

UNS8 COBtoPDO(int COB) {
	return (UNS8) 0;
}

void init_model() {
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	pthread_mutex_init(&MODEL_lock, &attr);
	pthread_mutex_init(&REPORTED_DATA_lock, &attr);
	pthread_mutex_init(&send_queue_COB_lock, &attr);
}

void debug_test() {
	int result = 0;
	UNS16 index = 0x400b;
	UNS8 subindex = 0x21;
	UNS8 datatype = octet_string;

	result = readNetworkDictCallback(&actuators_Data, 0xB, (UNS16) index,
			(UNS8) subindex, (UNS8) datatype, CheckReadSDO, 1);

	my_sleep(1);

	INTEGER64 data = 6666;
	UNS32 size = sizeof(data);
	// fill our super variable
	result = writeLocalDict(&actuators_Data, 0x5000, 0x0, &data, &size, 0);

	printf("result = 0x%04X\n", result);
	sendOnePDOevent(&actuators_Data, 0x0);

	data = 7777;
	result = writeLocalDict(&actuators_Data, 0x5000, 0x0, &data, &size, 0);
	sendOnePDOevent(&actuators_Data, 0x0);

	subindex = 0x20;
	result = readNetworkDictCallback(&actuators_Data, 0xB, (UNS16) index,
			(UNS8) subindex, (UNS8) datatype, CheckReadSDO, 1);

}

int main(int argc, char **argv) {
	int result = 0;

	error_log = fopen("error.log", "a");
	if (!error_log) {
		perror("error_log open:");
		exit(1);
	}
	setvbuf(error_log, NULL, _IONBF, 0);

	CANopen_startup();

	init_model();

#if DEBUG
	debug_test();
#endif

	while (true) {
		for (int i = 0; i < sizeof(pollable_entries) / sizeof(pollable_entries[0]); i++) {
			UNS8 nodeId = pollable_entries[i].nodeId;
			UNS16 index = pollable_entries[i].index;
			UNS8 subindex = pollable_entries[i].subindex;
			UNS8 datatype = pollable_entries[i].datatype;

			result = readNetworkDictCallback(&actuators_Data, 0xB/*nodeId*/,
					index, subindex, datatype, CheckReadSDO, SDO_USE_BLOCK_MODE);
			usleep(100);
		}

		// some here must be SYNC half period waiting
		usleep(1000);

		/* drain send queue when appropriate -- maybe at start of SYNC period */
		while (!send_queue_COB.empty()) {
			SEND_QUEUE_LOCK();
			sendOnePDOevent(&actuators_Data, COBtoPDO(send_queue_COB.front()));
			send_queue_COB.pop();
			SEND_QUEUE_UNLOCK();
		}

	}

	pause();

	CANopen_shutdown();

	fclose(error_log);

	return 0;
}
