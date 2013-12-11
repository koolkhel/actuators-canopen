#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <getopt.h>

#include <stdio.h>
#include <canfestival.h>
#include <data.h>
#include <sys/select.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>

#include "./actuators.h"
#include "./model.h"
#include "./canopen-util.h"
#include "./canopen-data.h"
#include "./actuators-ros.h"

#include "./matlab-connector.h"

// #include <ros/ros.h>

#include <assert.h>

static char busname[] = "1";
static char baudrate[] = "500K";

s_BOARD Board = { "", "" };
FILE *error_log = NULL;

pthread_t ros_thread;
pthread_t expedited_sdo_thread;
pthread_t matlab_thread;

sig_atomic_t exit_flag = 0;

int pipe_pdo_read = 0;
int pipe_pdo_write = 0;

enum actuators_mode_t {
	ACTUATORS_MAIN = 1,
	ACTUATORS_FAILSAFE = 2,
	ACTUATORS_FAILSAFE_EMERGENCY = 3,
};

int actuators_mode = ACTUATORS_MAIN;
volatile int GLOBAL_PDO_ENABLED = 1;
volatile int GLOBAL_SDO_ENABLED = 1;

struct option options[] = {
        {"type"   , 1, NULL, static_cast<int>('t')},
};

int callback_no = 0;

struct actuators_model MODEL;
extern pthread_mutex_t MODEL_lock;

struct Indigo_OD_Callback callbacks[200];

int pdo_callback_no = 0;

struct PDO_callback pdo_callbacks[512];

void set_actuators_mode(int _actuators_mode) {
	actuators_mode = _actuators_mode;

	switch (actuators_mode) {
	case ACTUATORS_FAILSAFE_EMERGENCY:
	case ACTUATORS_MAIN:
		GLOBAL_PDO_ENABLED = 1;
		GLOBAL_PDO_ENABLED = 1;
		break;
	case ACTUATORS_FAILSAFE:
		GLOBAL_PDO_ENABLED = 0;
		GLOBAL_SDO_ENABLED = 0;
		break;
	default:
		exit(1);
	}
}

static void parse_options(int argc, char **argv) {
	int opt = 0;
	while ((opt = getopt_long(argc, argv, "t:", options, NULL)) != -1) {
		switch (opt) {
		case 't':
			if (!strcmp(optarg, "main")) {
				set_actuators_mode(ACTUATORS_MAIN);
			} else if (!strcmp(optarg, "failsafe")) {
				set_actuators_mode(ACTUATORS_FAILSAFE);
			} else {
				fprintf(stderr, "-t can be main or failsafe only\n");
				exit(1);
			}
			break;
		case '?':
			break;
		default:
			break;
		}
	}
}

/**************************** INIT ********************************************/
void Init(CO_Data *d, UNS32 id) {
	(void) d;
	(void) id;
	if (Board.baudrate) {
		/* Init node state*/
		setState(&actuators_Data, Initialisation);
	}
}

/***************************  CLEANUP  *****************************************/
void Exit(CO_Data* d, UNS32 nodeid) {
	(void) d;
	(void) nodeid;
	if (strcmp(Board.baudrate, "none")) {
		/* Stop master */
		setState(&actuators_Data, Stopped);
	}
}

#define RECEIVE(VALUE, NAME, SCALE) do {\
			MODEL.NAME = VALUE / SCALE; \
			fprintf(stderr, "receiving %s\n", # NAME);\
		} while(0)

#define RECEIVE_PRINT(VALUE, NAME, SCALE, FORMAT) do {\
			MODEL.NAME = VALUE / SCALE; \
			printf("receiving " # NAME " value " # FORMAT "\n", MODEL.NAME);\
		} while(0)

#define RECEIVE_ARRAY(VALUE, NAME) do {\
			memcpy(&MODEL.NAME,&VALUE,sizeof(VALUE)); \
			fprintf(stderr, "receiving %s\n", # NAME);\
		} while(0)


#define DECLARE_M(INDEX, SUBINDEX) \
	if (size != sizeof(struct data_## INDEX ## _## SUBINDEX)) {\
		fprintf(error_log, "%ld size mismatch: %d against reference %d aborting\n", time(NULL), size, sizeof(struct data_## INDEX ## _## SUBINDEX));\
		printf("%ld size mismatch: %d against reference %d for index 0x" #INDEX " subindex 0x" #SUBINDEX "\n", time(NULL), size, sizeof(struct data_## INDEX ## _## SUBINDEX));\
	};\
	struct data_## INDEX ##_## SUBINDEX *M = (struct data_ ## INDEX ##_## SUBINDEX *) data

#define DECLARE_EMCY(NAME, CODE) \
	union emcy_## NAME ##_## CODE *emcy = (union emcy_## NAME ##_## CODE *) data

// assert(size == sizeof(struct data_## INDEX ## _## SUBINDEX));

#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

#define RECEIVE_DEBUG(VALUE, NAME, SCALE, FORMAT) do {\
	MODEL.NAME = VALUE; \
	fprintf(stdout,"receiving " STRINGIFY(NAME) " : " FORMAT "", MODEL.NAME);\
	fprintf(stdout, " with raw bytes : "); \
	int offset = offsetof(typeof(M), NAME);\
	print_hex_bytes(((unsigned char *)&M + offset, sizeof(M.NAME));\
	fprintf(stdout, "\n");\
} while (0)

extern int left_engine_startstop_command_sent;
PDO_CALLBACK(0x5100, 0x188, left_motor_startstop_response) {
        DECLARE_PDO_CALLBACK_VARS;

        char new_data[8] = {0};
        unsigned int send_size = 0;

        readLocalDict(d, 0x5100, 0x0, data, &size, &dataType, 0);

        switch (Subindex) {
        case 0x0:
        	if (left_engine_startstop_command_sent) {
        		printf("PARSED PDO left_motor callback result 0x188: ");
        		print_hex((char *) data, size);
        		printf("\n");

        		// confirmation
        		if (data[0] == 0xa0) {
        			new_data[0] = 0xb0;
        		} else if (data[0] == 0x55) {
        			new_data[0] = 0x66;
        		}
        		send_size = 8;
        		writeLocalDict(&actuators_Data, 0x5010, 0x0, &new_data[0], &send_size, 0);
        		enqueue_PDO(0x10);
        		left_engine_startstop_command_sent = 0;
        	}
        	break;
        default:
        	printf("unknown PDO subindex %.02hhX\n", Subindex);
        	break;
        }

        return OD_SUCCESSFUL;
}

extern volatile int right_engine_startstop_command_sent;
PDO_CALLBACK(0x5101, 0x189, right_motor_startstop_response) {
        DECLARE_PDO_CALLBACK_VARS;

        static unsigned char new_data[8] = {0};
        unsigned int send_size = 0;

        readLocalDict(d, 0x5101, 0x0, data, &size, &dataType, 0);

        switch (Subindex) {
        case 0x0:
        	if (right_engine_startstop_command_sent) {
        		printf("PARSED PDO right_motor callback result 0x189: ");
        		print_hex((char *) data, size);
        		printf("\n");

        		// confirmation
        		if (data[0] == 0xa0) {
        			new_data[0] = 0xb0;
        		} else if (data[0] == 0x55) {
        			new_data[0] = 0x66;
        		}
        		send_size = 8;
        		writeLocalDict(&actuators_Data, 0x5011, 0x0, &new_data[0], &send_size, 0);
        		enqueue_PDO(0x11);
        		right_engine_startstop_command_sent = 0;
        	}
        	break;
        default:
        	printf("unknown PDO subindex %.02hhX\n", Subindex);
        	break;
        }

        return OD_SUCCESSFUL;
}

extern volatile int switch_helium_valve_command_sent;
PDO_CALLBACK(0x5102, 0x182, helium_valve_response) {
        DECLARE_PDO_CALLBACK_VARS;

        unsigned char new_data[8] = {0};
        unsigned int send_size = 0;

        readLocalDict(d, 0x5102, 0x0, data, &size, &dataType, 0);

        switch (Subindex) {
        case 0x0:
        	printf("PARSED PDO helium valve callback result 0x182: ");
        	print_hex((char *) data, size);
        	printf("\n");

        	if (switch_helium_valve_command_sent) {
        		// confirmation
        		if (data[0] == 0xaa) {
        			new_data[0] = 0xbb;
        		} else if (data[0] == 0x55) {
        			new_data[0] = 0x66;
        		}
        		send_size = 8;
        		writeLocalDict(&actuators_Data, 0x5018, 0x0, &new_data[0], &send_size, 0);
        		enqueue_PDO(0x18);
        		switch_helium_valve_command_sent = 0;
        	}
        	break;
        default:
        	printf("unknown PDO subindex %.02hhX\n", Subindex);
        	break;
        }

        return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5103, 0x204, remote_control_switch) {
	DECLARE_PDO_CALLBACK_VARS;

	readLocalDict(d, 0x5103, 0x0, data, &size, &dataType, 0);

    switch (Subindex) {
    case 0x0:
    	printf("PARSED Remote control switch callback result 0x204: ");
    	print_hex((char *) data, size);
    	printf("\n");

    	// confirmation
    	if (data[0] == 0xAA) {
    		// switch to remote control
    		if (MODEL.remote_control_allowed_state && !MODEL.remote_control_enabled) {
    			fprintf(stderr, "remote control enabled!\n");
    			MODEL.remote_control_enabled = 1;
    		} else if (!MODEL.remote_control_allowed_state) {
        		fprintf(stderr, "remote control requested, but not allowed!\n");
    		}
    	} else if (data[0] == 0x55) {
    		// switch back
    		if (MODEL.remote_control_enabled) {
    			fprintf(stderr, "remote control disabled!\n");
    			MODEL.remote_control_enabled = 0;
    		}
    	}
    	break;
    default:
    	printf("unknown PDO subindex %.02hhX\n", Subindex);
    	break;
    }


	return OD_SUCCESSFUL;
}
//////////////////////////////////////////////////////
//                    EMCY
//////////////////////////////////////////////////////

PDO_CALLBACK(0x5110, 0x804, emcy_load) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(load, 804);

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5111, 0x806, emcy_control_surface) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(control_surface, 806);

	readLocalDict(d, 0x5111, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->left_horizontal_control_surface_1_status, left_horizontal_control_surface_1_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_horizontal_control_surface_2_status, left_horizontal_control_surface_2_status, 1, "%hhu");

	RECEIVE_PRINT(emcy->right_horizontal_control_surface_3_status, right_horizontal_control_surface_3_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_horizontal_control_surface_4_status, right_horizontal_control_surface_4_status, 1, "%hhu");

	RECEIVE_PRINT(emcy->left_vertical_control_surface_5_status, left_vertical_control_surface_5_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_vertical_control_surface_6_status, left_vertical_control_surface_6_status, 1, "%hhu");

	RECEIVE_PRINT(emcy->right_vertical_control_surface_7_status, right_vertical_control_surface_7_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_vertical_control_surface_8_status, right_vertical_control_surface_8_status, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5112, 0x807, emcy_tail_electromotor) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(tail_electromotor, 807);

	readLocalDict(d, 0x5112, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->left_electromotor_servo_status, left_electromotor_servo_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_electromotor_servo_status, right_electromotor_servo_status, 1, "%hhu");

	RECEIVE_PRINT(emcy->left_electromotor_status, left_electromotor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_electromotor_status, right_electromotor_status, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5113, 0x808, emcy_left_main_engine) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(left_main_engine, 808);

	readLocalDict(d, 0x5113, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->left_main_engine_failure_code, left_main_engine_failure_code, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_failure_code_2, left_main_engine_failure_code_2, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_aux_fuel_tank_sensor_status, left_main_engine_aux_fuel_tank_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_cylinder_1_temperature_sensor_status, left_main_engine_cylinder_1_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_cylinder_2_temperature_sensor_status, left_main_engine_cylinder_2_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_exhaust_temperature_sensor_status, left_main_engine_exhaust_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_fuel_pressure_sensor_status, left_main_engine_fuel_pressure_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_initialization_status, left_main_engine_initialization_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_main_fuel_tank_sensor_status, left_main_engine_main_fuel_tank_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_shutdown_status, left_main_engine_shutdown_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_temperature_sensor_status, left_main_engine_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_throttle_servo_status, left_main_engine_throttle_servo_status, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5114, 0x809, emcy_right_main_engine) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(right_main_engine, 809);

	readLocalDict(d, 0x5114, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->right_main_engine_failure_code, right_main_engine_failure_code, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_failure_code_2, right_main_engine_failure_code_2, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_aux_fuel_tank_sensor_status, right_main_engine_aux_fuel_tank_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_cylinder_1_temperature_sensor_status, right_main_engine_cylinder_1_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_cylinder_2_temperature_sensor_status, right_main_engine_cylinder_2_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_exhaust_temperature_sensor_status, right_main_engine_exhaust_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_fuel_pressure_sensor_status, right_main_engine_fuel_pressure_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_initialization_status, right_main_engine_initialization_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_main_fuel_tank_sensor_status, right_main_engine_main_fuel_tank_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_shutdown_status, right_main_engine_shutdown_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_temperature_sensor_status, right_main_engine_temperature_sensor_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_throttle_servo_status, right_main_engine_throttle_servo_status, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5115, 0x80a, emcy_left_main_engine_servo) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(left_main_engine_servo, 80A);

	readLocalDict(d, 0x5115, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->left_main_engine_servo_communication_link_status, left_main_engine_servo_communication_link_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_servo_dc_current_status, left_main_engine_servo_dc_current_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_servo_dc_voltage_status, left_main_engine_servo_dc_voltage_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_servo_failure_code, left_main_engine_servo_failure_code, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_servo_mechanical_structure_status, left_main_engine_servo_mechanical_structure_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_servo_sensor_failure, left_main_engine_servo_sensor_failure, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_main_engine_servo_temperature_sensor_status, left_main_engine_servo_temperature_sensor_status, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5116, 0x80b, emcy_right_main_engine_servo) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(right_main_engine_servo, 80B);

	readLocalDict(d, 0x5116, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->right_main_engine_servo_communication_link_status, right_main_engine_servo_communication_link_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_servo_dc_current_status, right_main_engine_servo_dc_current_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_servo_dc_voltage_status, right_main_engine_servo_dc_voltage_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_servo_failure_code, right_main_engine_servo_failure_code, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_servo_mechanical_structure_status, right_main_engine_servo_mechanical_structure_status, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_servo_sensor_failure, right_main_engine_servo_sensor_failure, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_main_engine_servo_temperature_sensor_status, right_main_engine_servo_temperature_sensor_status, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5117, 0x80c, emcy_left_ballonet) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(left_ballonet, 80C);

	readLocalDict(d, 0x5117, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->left_ballonet_failure_code, left_ballonet_failure_code, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_ballonet_fan_possible, left_ballonet_fan_possible, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_ballonet_valve_close_possible, left_ballonet_valve_close_possible, 1, "%hhu");
	RECEIVE_PRINT(emcy->left_ballonet_valve_open_possible, left_ballonet_valve_open_possible, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5118, 0x80d, emcy_right_ballonet) {
	DECLARE_PDO_CALLBACK_VARS;

	DECLARE_EMCY(right_ballonet, 80D);

	readLocalDict(d, 0x5118, 0x0, data, &size, &dataType, 0);

	RECEIVE_PRINT(emcy->right_ballonet_failure_code, right_ballonet_failure_code, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_ballonet_fan_possible, right_ballonet_fan_possible, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_ballonet_valve_close_possible, right_ballonet_valve_close_possible, 1, "%hhu");
	RECEIVE_PRINT(emcy->right_ballonet_valve_open_possible, right_ballonet_valve_open_possible, 1, "%hhu");

	return OD_SUCCESSFUL;
}

PDO_CALLBACK(0x5119, 0x80e, emcy_helium_valve) {
	DECLARE_PDO_CALLBACK_VARS;

	return OD_SUCCESSFUL;
}


////////////////////////////////////////////////////////
//            EMCY end
////////////////////////////////////////////////////////

void notify_ros_topic(UNS16 index, UNS8 subindex) {
	int i = 0;

	for (i = 0; i < callback_no; i++) {
		if (callbacks[i].index == index && callbacks[i].subindex == subindex) {
			callbacks[i].semaphore = 1;
			break;
		}
	}
}

// EMERGENCY
CALLBACK(0x4002, 0x10) {
	notify_ros_topic(0x4002, 0x10);
}

CALLBACK(0x4002, 0x20) {
	// failure_code
	DECLARE_M(4002, 20);

	//RECEIVE_ARRAY(M->failure_code, failure_code);

	notify_ros_topic(0x4002, 0x20);
}

CALLBACK(0x4002, 0x21) {
	// communications_state
	DECLARE_M(4002, 21);

	//RECEIVE_ARRAY(M->link_status, link_status);

	notify_ros_topic(0x4002, 0x21);
}

CALLBACK(0x4002, 0x22) {
	DECLARE_M(4002, 22);
	// power_system_state
	//RECEIVE_ARRAY(M->power_status, power_status);

	notify_ros_topic(0x4002, 0x22);
}

CALLBACK(0x4002, 0x23) {
	// left right motors
	DECLARE_M(4002, 23);

	//RECEIVE_ARRAY(M->main_engine_status, main_engine_status);

	notify_ros_topic(0x4002, 0x23);
}

CALLBACK(0x4002, 0x24) {
	// rotation state
	DECLARE_M(4002, 24);

	//RECEIVE_ARRAY(M->rotation_status, rotation_status);

	notify_ros_topic(0x4002, 0x24);
}

CALLBACK(0x4002, 0x25) {
	// misc state
	DECLARE_M(4002, 25);

	//RECEIVE_ARRAY(M->misc_status, misc_status);

	notify_ros_topic(0x4002, 0x25);
}

// ENVIRONMENT
CALLBACK(0x4003, 0x10) {
	notify_ros_topic(0x4003, 0x10);
}

CALLBACK(0x4003, 0x20) {
	DECLARE_M(4003, 20);

	notify_ros_topic(0x4003, 0x20);
}

// PAYLOAD_BLOCK
CALLBACK(0x4004, 0x10) {
	notify_ros_topic(0x4004, 0x10);
}

CALLBACK(0x4004, 0x20) {
	DECLARE_M(4004, 20);

	notify_ros_topic(0x4004, 0x20);
}

// POWER
CALLBACK(0x4005, 0x10) {
	notify_ros_topic(0x4005, 0x10);
}

CALLBACK(0x4005, 0x20) {
	notify_ros_topic(0x4005, 0x20);
}

// POWER CONTROL
CALLBACK(0x4005, 0x21) {
	DECLARE_M(4005, 21);

	// 0.1A
	RECEIVE_PRINT(M->_01_left_main_engine_generator_output_current, left_main_engine_generator_output_current, 10.0, "%f A");

	// 0.1V
	RECEIVE_PRINT(M->_02_left_main_engine_generator_output_voltage, left_main_engine_generator_output_voltage, 10.0, "%f V");

	// 0.1A
	RECEIVE_PRINT(M->_03_right_main_engine_generator_output_current, right_main_engine_generator_output_current, 10.0, "%f A");

	// 0.1V
	RECEIVE_PRINT(M->_04_right_main_engine_generator_output_voltage, right_main_engine_generator_output_voltage, 10.0, "%f A");

	// 0.1A
	RECEIVE_PRINT(M->_05_left_ballonet_control_current, left_ballonet_control_current, 10.0, "%f A");

	// 0.1A
	RECEIVE_PRINT(M->_06_right_ballonet_control_current, right_ballonet_control_current, 10.0, "%f A");

	// 0,1A
	RECEIVE_PRINT(M->_07_power_section_28v_power_equipment_current, power_section_28v_power_equipment_current, 10.0, "%f A");

	// 0,1A
	RECEIVE_PRINT(M->_08_tail_section_28v_power_equipment_current, tail_section_28v_power_equipment_current, 10.0, "%f A");

	// 0.1V
	RECEIVE_PRINT(M->_09_bus_backup_28v_voltage, bus_backup_28v_voltage, 10.0, "%f V");

	// 0,1V
	RECEIVE_PRINT(M->_10_failsafe_28v_voltage, failsafe_28v_voltage, 10.0, "%f V");

	// 0.1V
	RECEIVE_PRINT(M->_11_backup_battery_voltage, backup_battery_voltage, 10.0, "%f V");

	// 0,1V
	RECEIVE_PRINT(M->_12_failsafe_battery_voltage, failsafe_battery_voltage, 10.0, "%f V");

	// 0.1A
	RECEIVE_PRINT(M->_13_backup_battery_charge_current, backup_battery_charge_current, 10.0, "%f A");

	// 0.1A
	RECEIVE_PRINT(M->_14_backup_battery_discharge_current, backup_battery_discharge_current, 10.0, "%f A");

	// 1C
	RECEIVE_PRINT(M->_15_backup_battery_monoblock_temperature, backup_battery_monoblock_temperature, 10.0, "%f C");

	// примечание 1
	MODEL.backup_battery_failure_state = M->_16_backup_battery_failure_state;

	// 0.1A
	RECEIVE_PRINT(M->_17_failsafe_battery_charge_current, failsafe_battery_charge_current, 10.0, "%f A");

	// 0.1A
	RECEIVE_PRINT(M->_18_failsafe_battery_discharge_current, failsafe_battery_discharge_current, 10.0, "%f A");

	// 1C
	RECEIVE_PRINT(M->_19_failsafe_battery_monoblock_temperature, failsafe_battery_monoblock_temperature, 10.0, "%f C");

	// примечание 2
	MODEL.failsafe_battery_failure_state = M->_20_failsafe_battery_failure_state;

	// 0.1A
	RECEIVE_PRINT(M->_21_left_charge_device_output_current, left_charge_device_output_current, 10.0, "%f A");

	// reserved
	RECEIVE_PRINT(M->_22_left_charge_device_state, left_charge_device_state, 1.0, "%f charge state");

	// 0.1A
	RECEIVE_PRINT(M->_23_right_charge_device_output_current, right_charge_device_output_current, 10.0, "%f A");

	// reserved
	RECEIVE_PRINT(M->_24_right_charge_device_state, right_charge_device_state, 1.0, "%f charge state");

	// примечание 3
	MODEL.distribution_relay_state = M->_25_distribution_relay_state;

	// 0.1A
	RECEIVE_PRINT(M->_26_load_equipment_28v_current, load_equipment_28v_current, 10.0, "%f A");

	// примечание 4
	MODEL.distribution_lines_failure_state = M->_27_distribution_lines_failure_state;

	notify_ros_topic(0x4005, 0x21);
}

CALLBACK(0x4006, 0x20) {
	DECLARE_M(4006, 20);

	RECEIVE_PRINT(M->left_control_surface_X, left_control_surface_X, 100.0, "%f degree");
	RECEIVE_PRINT(M->left_control_surface_Y, left_control_surface_Y, 100.0, "%f degree");

	RECEIVE_PRINT(M->right_control_surface_X, right_control_surface_X, 100.0, "%f degree");
	RECEIVE_PRINT(M->right_control_surface_Y, right_control_surface_Y, 100.0, "%f degree");

	notify_ros_topic(0x4006, 0x20);
}

CALLBACK(0x4006, 0x21) {
	DECLARE_M(4006, 21);

	RECEIVE_PRINT(M->control_surface_1_current, control_surface_1_current, 10.0, "%f A");
	RECEIVE_PRINT(M->control_surface_2_current, control_surface_2_current, 10.0, "%f A");
	RECEIVE_PRINT(M->control_surface_3_current, control_surface_3_current, 10.0, "%f A");
	RECEIVE_PRINT(M->control_surface_4_current, control_surface_4_current, 10.0, "%f A");

	RECEIVE_PRINT(M->control_surface_5_current, control_surface_5_current, 10.0, "%f A");
	RECEIVE_PRINT(M->control_surface_6_current, control_surface_6_current, 10.0, "%f A");
	RECEIVE_PRINT(M->control_surface_7_current, control_surface_7_current, 10.0, "%f A");
	RECEIVE_PRINT(M->control_surface_8_current, control_surface_8_current, 10.0, "%f A");

	notify_ros_topic(0x4006, 0x21);
}

// TAIL_ENGINE
CALLBACK(0x4007, 0x10) {
	notify_ros_topic(0x4007, 0x10);
}


CALLBACK(0x4007, 0x20) {
	DECLARE_M(4007, 20);

	RECEIVE_PRINT(M->left_electromotor_angle_X, left_electromotor_angle_X, 100.0, "%f");
	RECEIVE_PRINT(M->left_electromotor_angle_Y, left_electromotor_angle_Y, 100.0, "%f");
	RECEIVE_PRINT(M->left_electromotor_rate, left_electromotor_rate, 1, "%d");

	RECEIVE_PRINT(M->right_electromotor_angle_X, right_electromotor_angle_X, 100.0, "%f");
	RECEIVE_PRINT(M->right_electromotor_angle_Y, right_electromotor_angle_Y, 100.0, "%f");
	RECEIVE_PRINT(M->right_electromotor_rate, right_electromotor_rate, 1, "%d");

	QUEUE_MATLAB_NOTIFY(left_electromotor_rate, "left_electromotor_rate %hu");
	QUEUE_MATLAB_NOTIFY(right_electromotor_rate, "right_electromotor_rate %hu");
	QUEUE_MATLAB_NOTIFY(left_electromotor_angle_Y, "left_electromotor_angle_Y %f");
	QUEUE_MATLAB_NOTIFY(right_electromotor_angle_Y, "right_electromotor_angle_Y %f");
	QUEUE_MATLAB_NOTIFY(left_electromotor_angle_X, "left_electromotor_angle_X %f");
	QUEUE_MATLAB_NOTIFY(right_electromotor_angle_X, "right_electromotor_angle_X %f");

	notify_ros_topic(0x4007, 0x20);
}

// electromotor diagnostics
CALLBACK(0x4007, 0x21) {
	DECLARE_M(4007, 21);

	RECEIVE_PRINT(M->left_electromotor_voltage, left_electromotor_voltage, 10.0, "%f V");

		// / 10 V
	RECEIVE_PRINT(M->right_electromotor_voltage, right_electromotor_voltage, 10.0, "%f V");

		// / 10 A
	RECEIVE_PRINT(M->left_electromotor_current, left_electromotor_current, 10.0, "%f A");

		// / 10 A
	RECEIVE_PRINT(M->right_electromotor_current, right_electromotor_current, 10.0, "%f A");

		// / 10 C
	RECEIVE_PRINT(M->left_electromotor_temperature, left_electromotor_temperature, 10.0, "%f C");

		// / 10 C
	RECEIVE_PRINT(M->right_electromotor_temperature, right_electromotor_temperature, 10.0, "%f C");

		// / 10 C
	RECEIVE_PRINT(M->left_electromotor_rotation_temperature, left_electromotor_rotation_temperature, 10.0, "%f C");

		// / 10 C
	RECEIVE_PRINT(M->right_electromotor_rotation_temperature, right_electromotor_rotation_temperature, 10.0, "%f C");

	notify_ros_topic(0x4007, 0x21);
}

// LEFT MOTOR
CALLBACK(0x4008, 0x10) {
	notify_ros_topic(0x4008, 0x10);
}

CALLBACK(0x4008, 0x20) {
	DECLARE_M(4008, 20);

	// 1 rpm
	RECEIVE_PRINT(M->left_main_engine_rate, left_main_engine_rate, 1, "%hu RPM");

	QUEUE_MATLAB_NOTIFY(left_main_engine_rate, "left_main_engine_rate %hu");

	// 0.1%
	RECEIVE_PRINT(M->left_main_engine_fuel_level, left_main_engine_fuel_level, 10.0, "%f %%");

	// 0.1%
	RECEIVE_PRINT(M->left_main_engine_aux_fuel_level, left_main_engine_aux_fuel_level, 10.0, "%f %%");

	// 0.1 C
	RECEIVE_PRINT(M->left_main_engine_exhaust_temperature, left_main_engine_exhaust_temperature, 10.0, "%f C");

	// 0.1 C
	RECEIVE_PRINT(M->left_main_engine_cylinder_1_temperature, left_main_engine_cylinder_1_temperature, 10.0, "%f C");

	// 0.1 C
	RECEIVE_PRINT(M->left_main_engine_cylinder_2_temperature, left_main_engine_cylinder_2_temperature, 10.0, "%f C");

	// 0.1 degree
	RECEIVE_PRINT(M->left_main_engine_throttle_1_angle, left_main_engine_throttle_1_angle, 10.0, "%f degrees");

	// 0.1 degree
	RECEIVE_PRINT(M->left_main_engine_throttle_2_angle, left_main_engine_throttle_2_angle, 10.0, "%f degrees");

	QUEUE_MATLAB_NOTIFY(left_main_engine_throttle_1_angle, "left_main_engine_throttle_1_angle %f");
	QUEUE_MATLAB_NOTIFY(left_main_engine_throttle_2_angle, "left_main_engine_throttle_2_angle %f");

	MODEL.left_main_engine_relay_state = M->left_main_engine_relay_state;

	notify_ros_topic(0x4008, 0x20);
}

// RIGHT MOTOR
CALLBACK(0x4009, 0x10) {
	notify_ros_topic(0x4009, 0x10);
}

CALLBACK(0x4009, 0x20) {
	DECLARE_M(4009, 20);

	RECEIVE_PRINT(M->right_main_engine_rate, right_main_engine_rate, 1, "%hu RPM");

	QUEUE_MATLAB_NOTIFY(right_main_engine_rate, "right_main_engine_rate %hu");

	// 0.1%
	RECEIVE_PRINT(M->right_main_engine_fuel_level, right_main_engine_fuel_level, 10.0, "%f %%");

	// 0.1%
	RECEIVE_PRINT(M->right_main_engine_aux_fuel_level, right_main_engine_aux_fuel_level, 10.0, "%f %%");

	// 0.1 C
	RECEIVE_PRINT(M->right_main_engine_exhaust_temperature, right_main_engine_exhaust_temperature, 10.0, "%f C");

	// 0.1 C
	RECEIVE_PRINT(M->right_main_engine_cylinder_1_temperature, right_main_engine_cylinder_1_temperature, 10.0, "%f C");

	// 0.1 C
	RECEIVE_PRINT(M->right_main_engine_cylinder_2_temperature, right_main_engine_cylinder_2_temperature, 10.0, "%f C");

	// 0.1 degree
	RECEIVE_PRINT(M->right_main_engine_throttle_1_angle, right_main_engine_throttle_1_angle, 10.0, "%f degrees");

	// 0.1 degree
	RECEIVE_PRINT(M->right_main_engine_throttle_2_angle, right_main_engine_throttle_2_angle, 10.0, "%f degrees");

	QUEUE_MATLAB_NOTIFY(right_main_engine_throttle_1_angle, "right_main_engine_throttle_1_angle %f");
	QUEUE_MATLAB_NOTIFY(right_main_engine_throttle_2_angle, "right_main_engine_throttle_2_angle %f");

	MODEL.right_main_engine_relay_state = M->right_main_engine_relay_state;

	notify_ros_topic(0x4009, 0x20);
}

// LEFT_MOTOR_ROTATE
CALLBACK(0x400a, 0x10) {
	notify_ros_topic(0x4009, 0x10);
}

CALLBACK(0x400a, 0x20) {
	DECLARE_M(400a, 20);

	RECEIVE_PRINT(M->left_main_engine_rotation_angle, left_main_engine_rotation_angle, 100.0, "%f degrees");

	QUEUE_MATLAB_NOTIFY(left_main_engine_rotation_angle, "left_main_engine_rotation_angle %f");

	notify_ros_topic(0x400a, 0x20);
}

CALLBACK(0x400a, 0x21) {
	DECLARE_M(400a, 21);

	RECEIVE_PRINT(M->left_main_engine_rotation_current, left_main_engine_rotation_current, 10.0, "%f A");
	RECEIVE_PRINT(M->left_main_engine_rotation_voltage, left_main_engine_rotation_voltage, 10.0, "%f V");

	notify_ros_topic(0x400a, 0x21);
}

// RIGHT_MOTOR_ROTATE
CALLBACK(0x400b, 0x20) {
	DECLARE_M(400b, 20);

	RECEIVE_PRINT(M->right_main_engine_rotation_angle, right_main_engine_rotation_angle, 100.0, "%f degrees");

	QUEUE_MATLAB_NOTIFY(right_main_engine_rotation_angle, "right_main_engine_rotation_angle %f");

	notify_ros_topic(0x400b, 0x20);
}

CALLBACK(0x400b, 0x21) {
	DECLARE_M(400b, 21);

	RECEIVE_PRINT(M->right_main_engine_rotation_current, right_main_engine_rotation_current, 10.0, "%f A");
	RECEIVE_PRINT(M->right_main_engine_rotation_voltage, right_main_engine_rotation_voltage, 10.0, "%f V");

	notify_ros_topic(0x400b, 0x21);
}



CALLBACK(0x400c, 0x10) {
	notify_ros_topic(0x400c, 0x10);
};

CALLBACK(0x400c, 0x20) {
	DECLARE_M(400c, 20);

	RECEIVE_PRINT(M->left_ballonet_pressure_1, left_ballonet_pressure_1, 1, "%hu Pa");
	RECEIVE_PRINT(M->left_ballonet_pressure_2, left_ballonet_pressure_2, 1, "%hu Pa");
	RECEIVE_PRINT(M->left_ballonet_lights_state, left_ballonet_lights_state, 1, "%hu state");
	RECEIVE_PRINT(M->left_ballonet_control, left_ballonet_control, 1, "%hu");
	RECEIVE_PRINT(M->left_ballonet_linear_valve_resistance, left_ballonet_linear_valve_resistance, 1, "%f Om");
	RECEIVE_PRINT(M->left_ballonet_regime, left_ballonet_regime, 1, "%hx");

	notify_ros_topic(0x400c, 0x20);
};

CALLBACK(0x400d, 0x10) {
	notify_ros_topic(0x400d, 0x10);
}

CALLBACK(0x400d, 0x20) {
	DECLARE_M(400d, 20);

	RECEIVE_PRINT(M->right_ballonet_pressure_1, right_ballonet_pressure_1, 1, "%hu Pa");
	RECEIVE_PRINT(M->right_ballonet_pressure_2, right_ballonet_pressure_2, 1, "%hu Pa");
	RECEIVE_PRINT(M->right_ballonet_lights_state, right_ballonet_lights_state, 1, "%hu state");
	RECEIVE_PRINT(M->right_ballonet_control, right_ballonet_control, 1, "%hu");
	RECEIVE_PRINT(M->right_ballonet_linear_valve_resistance, right_ballonet_linear_valve_resistance, 1, "%f ???");
	RECEIVE_PRINT(M->right_ballonet_regime, right_ballonet_regime, 1, "%hx");

	notify_ros_topic(0x400d, 0x20);
}

CALLBACK(0x400e, 0x20) {
	DECLARE_M(400e, 20);

	RECEIVE_PRINT(M->helium_valve_open, helium_valve_open, 1, "%hu bool");

	notify_ros_topic(0x400e, 0x20);
}

int CallSDOCallback(UNS16 index, UNS8 subindex, UNS8 *data, UNS32 size) {
	UNS16 i = 0;
	int found = 0;
	for (i = 0; i < sizeof(callbacks) / sizeof(callbacks[0]); i++) {
		if ((callbacks[i].index == index)
				&& (callbacks[i].subindex == subindex)) {
			found = 1;
			(callbacks[i].callback_fn)(data, size);
			break;
		}
	}

	if (!found) {
		printf("Couldn't find callback for index 0x%hx subindex 0x%hhx\n", index, subindex);
	} else {
		printf("Found callback for index 0x%hx subindex 0x%hhx at i %hd address %p\n", index, subindex, i, callbacks[i].callback_fn);
	}

	return found;
}

#define INDIGO_SDO_BLOCK 1
#define INDIGO_SDO_EXPEDITED 2

#define MATLAB_ENABLED 1
#define MATLAB_DISABLED 0

/* regularily poll these remote OD entries */
struct pollable_OD_entry {
    UNS8 nodeId;
    UNS16 index;
    UNS8 subindex;
    UNS8 datatype;
    UNS8 transferType; // BLOCK or EXPEDITED

    // hold transfer data inside each entry
    UNS8 data[1024];
    UNS32 size;


    // 1 == matlab is enabled, real SDO is disabled
    // 0 == real SDO is enabled
    volatile int status;
};

struct pollable_OD_entry pollable_entries[] = {
	{0x07, 0x4007, 0x21, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // tail electromotor
	{0x07, 0x4007, 0x20, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // tail electromotor
    {0x05, 0x4005, 0x21, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // power system
    {0x08, 0x4008, 0x20, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // left main engine
    {0x09, 0x4009, 0x20, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // right main engine

    {0x0a, 0x400a, 0x20, octet_string, INDIGO_SDO_EXPEDITED, {0}, 1024, 0}, // left rotation
    {0x0a, 0x400a, 0x21, octet_string, INDIGO_SDO_EXPEDITED, {0}, 1024, 0}, // left rotation
    {0x0b, 0x400b, 0x20, octet_string, INDIGO_SDO_EXPEDITED, {0}, 1024, 0}, // right rotation
    {0x0b, 0x400b, 0x21, octet_string, INDIGO_SDO_EXPEDITED, {0}, 1024, 0}, // right rotation

	{0x0c, 0x400c, 0x20, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // ballonet left
    {0x0d, 0x400d, 0x20, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // ballonet right
    {0x0e, 0x400e, 0x20, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // helium valve

    {0x06, 0x4006, 0x20, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0}, // control surface
    {0x06, 0x4006, 0x21, octet_string, INDIGO_SDO_BLOCK, {0}, 1024, 0},
};

int pollable_entries_count = sizeof(pollable_entries) / sizeof(pollable_entries[0]);

void reset_matlab_feedback() {
	fprintf(stderr, "matlab disconnected, using real CAN data\n");
	for (int i = 0; i < pollable_entries_count; i++) {
		pollable_entries[i].status = MATLAB_DISABLED;
	}
}

void enable_matlab_feedback(UNS8 nodeid) {
	for (int i = 0; i < pollable_entries_count; i++) {
		if (pollable_entries[i].nodeId == nodeid) {
			if (pollable_entries[i].status == MATLAB_DISABLED) {
				pollable_entries[i].status = MATLAB_ENABLED;
				fprintf(stderr, "enabling matlab model for node 0x%hhx index 0x%hx subindex 0x%hhx\n",
					pollable_entries[i].nodeId, pollable_entries[i].index, pollable_entries[i].subindex);
			}
		}
	}
}

struct pollable_node {
	UNS8 nodeid;

    // each node goes in its own thread
    pthread_t sdo_thread;
} nodes[] = {
		{0x07, 0}, // tail electromotor
		{0x05, 0}, // power system
		{0x08, 0}, // left main engine
		{0x09, 0}, // right main engine
		{0x0a, 0}, // left main engine rotation
		{0x0b, 0}, // right main engine rotation
		{0x0c, 0}, // left ballonet
		{0x0d, 0}, // right ballonet
		{0x0e, 0}, // helium valve
		{0x06, 0}, // control surface
};

void *sdo_polling_thread(void *arg) {
	struct pollable_node *node = (struct pollable_node *) arg;
	int result = 0;
	UNS8 nodeid = node->nodeid;
	UNS32 abort_code;

	UNS8 pollable_entries_no = sizeof(pollable_entries) / sizeof(pollable_entries[0]);

	// for 10 hz polling
	pthread_mutex_t never_happens_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t never_happens_cond = PTHREAD_COND_INITIALIZER;
	struct timeval now_tv;
	struct timespec timeout;

#define NSEC_PER_SEC (1000 * 1000 * 1000)
#define SDO_HZ 10
	
	gettimeofday(&now_tv, NULL);

	timeout.tv_sec   = now_tv.tv_sec;
	timeout.tv_nsec += NSEC_PER_SEC / SDO_HZ; // add enough ms from now
	timeout.tv_sec  += timeout.tv_nsec / NSEC_PER_SEC;
	timeout.tv_nsec  = timeout.tv_nsec % NSEC_PER_SEC;

	while (!exit_flag) {
		for (UNS8 i = 0; i < pollable_entries_no; i++) {
			struct pollable_OD_entry *entry = &pollable_entries[i];

			if (!GLOBAL_SDO_ENABLED)
				continue;

			if (entry->nodeId != nodeid)
				continue;

			if (entry->status == MATLAB_ENABLED)
				continue;

			UNS16 index = entry->index;
			UNS8 subindex = entry->subindex;
			UNS32 datatype = entry->datatype;
			UNS32 transfer_type = entry->transferType;

			int timeout = 300 * 1000; // 300 ms

			result = 0;
			abort_code = 0;

			printf("sending %s SDO 0x%hhx 0x%hx 0x%hhx\n",
					transfer_type == INDIGO_SDO_BLOCK ? "block" : "expedited", nodeid, index, subindex);

			result = readNetworkDict(&actuators_Data, nodeid, index, subindex, datatype,
					transfer_type == INDIGO_SDO_BLOCK ? SDO_USE_BLOCK_MODE : SDO_USE_EXPEDITED_MODE);

			entry->size = 1024;
			memset(&entry->data[0], 0, entry->size);

			result = getReadResultNetworkDict(&actuators_Data, nodeid, &entry->data[0],
					&entry->size, &abort_code);

			while (((result == SDO_UPLOAD_IN_PROGRESS) || (result == SDO_BLOCK_UPLOAD_IN_PROGRESS)) && (timeout > 0)) {
				usleep(100);
				timeout -= 100;
				result = getReadResultNetworkDict(&actuators_Data, nodeid, &entry->data[0],
						&entry->size, &abort_code);
			}

			printf("result for SDO node 0x%hhx is %u\n", nodeid, result);

			if (result == SDO_FINISHED) {
				printf("SDO Result node 0x%hhx index 0x%hx subindex 0x%hhx: %d: ", nodeid, index, subindex, result);
				print_hex((char *) &entry->data[0], entry->size);
				printf("\n");
				CallSDOCallback(index, subindex, &entry->data[0], entry->size);
			} else {
				printf("SDO Result ABORT node 0x%hhx index 0x%hx subindex 0x%hhx code: %u\n", nodeid, index, subindex, abort_code);
			}
			closeSDOtransfer(&actuators_Data, nodeid, SDO_CLIENT);
		}

		pthread_mutex_lock(&never_happens_mutex);
		// ждём аккуратно сколько надо
		pthread_cond_timedwait(&never_happens_cond, &never_happens_mutex, &timeout);
		pthread_mutex_unlock(&never_happens_mutex);

		timeout.tv_nsec += NSEC_PER_SEC / SDO_HZ; // add enough ms
		timeout.tv_sec += timeout.tv_nsec / NSEC_PER_SEC;
		timeout.tv_nsec = timeout.tv_nsec % NSEC_PER_SEC;
	}

	return NULL;
}

std::queue<int> send_queue_COB;
pthread_mutex_t send_queue_COB_lock;

pthread_mutex_t MODEL_lock;
pthread_mutex_t REPORTED_DATA_lock;

void post_SlaveStateChange(CO_Data *data, UNS8 nodeId, e_nodeState newNodeState) {
	if (nodeId > sizeof(MODEL.node_status.block_status) / sizeof(MODEL.node_status.block_status[0]))
		return;

	switch (newNodeState) {
	case Initialisation:
		fprintf(stdout, "HEARTBEAT node %hhu state Initialisation\n", nodeId);
		break;
	case Disconnected:
		fprintf(stdout, "HEARTBEAT node %hhu state Disconnected\n", nodeId);
		MODEL.node_status.block_status[nodeId] = 1;

		if (nodeId == 1 && actuators_mode == ACTUATORS_FAILSAFE) {
			// 1 раз может быть не персил
			// с другой стороны, одноразовый таймер уже сработал и всё
			// с третьей стороны, если на шине что-то ещё есть, она появится
			// с четвёртой стороны, таймер 10 секунд в отличие от остальных, а 10 секунд уже надо реагировать

			set_actuators_mode(ACTUATORS_FAILSAFE_EMERGENCY);
		}
		break;
	case Preparing:
		fprintf(stdout, "HEARTBEAT node %hhu state Preparing/Connecting\n", nodeId);
		break;
	case Stopped:
		fprintf(stdout, "HEARTBEAT node %hhu state Stopped\n", nodeId);
		break;
	case Operational:
		fprintf(stdout, "HEARTBEAT node %hhu state Operational\n", nodeId);
		MODEL.node_status.block_status[nodeId] = 0;
		break;
	case Pre_operational:
		fprintf(stdout, "HEARTBEAT node %hhu state Pre_operational\n", nodeId);
		break;
	case Unknown_state:
		fprintf(stdout, "HEARTBEAT node %hhu state Unknown_state\n", nodeId);
		break;
	}

}

void BUS_CAN_HEARTBEAT_ERROR(CO_Data *data, UNS8 nodeId) {
	e_nodeState nodeState = getNodeState(data, nodeId);

	post_SlaveStateChange(data, nodeId, nodeState);
}

void post_sync(CO_Data *data) {
	if (GLOBAL_PDO_ENABLED) {
		sendOnePDOevent(data, 0x19);
	}
}

void CANopen_startup(void) {
	Board.busname = busname;
	Board.baudrate = baudrate;

	LoadCanDriver("/usr/local/lib/libcanfestival_can_kvaser.so");

	switch (actuators_mode) {
	case ACTUATORS_FAILSAFE:
		fprintf(stderr, "actuators FAILSAFE mode\n");
		setNodeId(&actuators_Data, 0x2);
		break;
	case ACTUATORS_MAIN:
	default:
		fprintf(stderr, "actuators MAIN mode\n");
		setNodeId(&actuators_Data, 0x1);
		break;
	}

	/* Init stack timer */
	TimerInit();

	canOpen(&Board, &actuators_Data);

	/* Start Timer thread */
	StartTimerLoop(&Init);

	setState(&actuators_Data, Pre_operational);

	my_sleep(1);

	setState(&actuators_Data, Operational);

	actuators_Data.post_sync = post_sync;

	// this part might still cause crashes
#if 0
	actuators_Data.heartbeatError = BUS_CAN_HEARTBEAT_ERROR;
	actuators_Data.post_SlaveStateChange = post_SlaveStateChange;
	heartbeatInit(&actuators_Data);
	lifeGuardInit(&actuators_Data);
#endif
}

void CANopen_shutdown(void) {
	//heartbeatStop(&actuators_Data);

	// Stop timer thread
	StopTimerLoop(&Exit);

	canClose(&actuators_Data);

	TimerCleanup();
}

void init_model() {
	memset(&MODEL, 0, sizeof(MODEL));
	MODEL.remote_control_enabled = 1; // enable remote control by default

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	pthread_mutex_init(&MODEL_lock, &attr);
	pthread_mutex_init(&REPORTED_DATA_lock, &attr);
	pthread_mutex_init(&send_queue_COB_lock, &attr);
}


/* ROS services use this for command mappings */
int enqueue_PDO(int PDO) {
	long int my_time = time(NULL);
	struct tm my_time_s;
	char buf[255];
	ssize_t bytes_written = 0;

	int allowed_PDOs[] = {
			0x19,  // REMOTE CONTROL ALLOWED
			0x04,  // LEFT_BALLONET_20C
			0x05,  // LEFT_BALLONET_30C
			0x06,  // LEFT_BALLONET_40C
			0x07,  // LEFT_BALLONET_50C
			0x08,  // RIGHT_BALLONET_20D
			0x09,  // RIGHT_BALLONET_30D
			0x0A,  // RIGHT_BALLONET_40D
			0x0B,  // RIGHT_BALLONET_50D
			0x14,  // LEFT_MOTOR_312
			0x15,  // RIGHT_MOTOR_313
			0x16,  // POWER_DISTRIBUTION_RELAY_305
			0x17,  // GENERATOR_205
	};

	bool found = false;

	if (!GLOBAL_PDO_ENABLED)
		return 1;

	localtime_r(&my_time, &my_time_s);
	asctime_r(&my_time_s, buf);

	for (int i = 0; i < sizeof(allowed_PDOs) / sizeof(allowed_PDOs[0]); i++) {
		if (allowed_PDOs[i] == PDO) {
			found = true;
			break;
		}
	}

	if (!MODEL.remote_control_enabled || found) {
		printf("%s QUEUEING PDO 0x%.02x...\n", buf, PDO);
		buf[0] = (UNS8) PDO;

		bytes_written = write(pipe_pdo_write, buf, 1);

	} else {
		printf("%s NOT QUEUEING PDO 0x%.02x DUE TO REMOTE CONTROL\n", buf, PDO);
	}

	if (bytes_written == 0)
		return 1;
	else
		return 0;
}

#define NO_PDO_IN_QUEUE 255
UNS8 _retreive_PDO_queue_entry() {
	UNS8 PDO = NO_PDO_IN_QUEUE;
	long int my_time = time(NULL);
	struct tm my_time_s;
	char buf[255];
	int result = 0;

	localtime_r(&my_time, &my_time_s);
	asctime_r(&my_time_s, buf);

	result = read(pipe_pdo_read, &PDO, 1);

	if (result == 1) {
		printf("%s SENDING PDO 0x%.02x...", buf, PDO);
		return PDO;
	} else
		return NO_PDO_IN_QUEUE;
}

void start_matlab_thread(int argc, char **argv) {
	int result = 0;

	/* we want SIGINT to be passed to matlab thread */
	sigset_t set;

	sigaddset(&set, SIGINT);
	result = pthread_sigmask(SIG_BLOCK, &set, NULL);
	if (result) {
		perror("pthread_sigmask");
		exit(1);
	}

	if ((result = pthread_create(&matlab_thread, NULL, matlab_main, NULL))) {
		fprintf(stderr, "couldn't create matlab thread: %s %d\n", strerror(result), result);
		exit(1);
	}
}


void start_ros_thread(int argc, char **argv) {
	int result;

	static struct actuators_ros_settings settings;
	settings.argc = argc;
	settings.argv = argv;

	/* we want SIGINT to be passed to can thread */
	sigset_t set;
	sigaddset(&set, SIGINT);
	result = pthread_sigmask(SIG_UNBLOCK, &set, NULL);
	if (result) {
		perror("pthread_sigmask");
		exit(1);
	}

	if ((result = pthread_create(&ros_thread, NULL, ros_main, &settings))) {
		fprintf(stderr, "couldn't create ros thread: %s %d\n", strerror(result), result);
		exit(1);
	}
}

void bye(int signum) {
	exit_flag = 1;
}

int main(int argc, char **argv) {
	int result = 0;
	int pipe_fds[2];

	parse_options(argc, argv);

	printf("starting actuators\n");
	printf("%d SDO callbacks\n", callback_no);
	for (int i = 0; i < callback_no; i++) {
		printf("%d SDO callback index 0x%hx subindex 0x%hhx address %p\n",
				i, callbacks[i].index, callbacks[i].subindex, callbacks[i].callback_fn);

		callbacks[i].semaphore = 0;
	}

	printf("%d PDO callbacks\n", pdo_callback_no);
	for (int i = 0; i < pdo_callback_no; i++) {
		RegisterSetODentryCallBack(&actuators_Data, pdo_callbacks[i].index, 0, pdo_callbacks[i].callback_fn);
	}

	result = pipe2(pipe_fds, O_NONBLOCK);
	if (result) {
		perror("pipe");
		exit(1);
	}

	pipe_pdo_read = pipe_fds[0];
	pipe_pdo_write = pipe_fds[1];

	signal(SIGINT, bye);

	error_log = fopen("error.log", "a");
	if (!error_log) {
		perror("error_log open:");
		exit(1);
	}
	setvbuf(error_log, NULL, _IONBF, 0);
	setlinebuf(stdout);

	CANopen_startup();

	init_model();

	start_ros_thread(argc, argv);
	start_matlab_thread(argc, argv);
		// SDO
		// III
    my_sleep(1);


    for (unsigned int i = 0; i < sizeof(nodes) / sizeof(nodes[0]); i++) {
        pthread_create(&nodes[i].sdo_thread, NULL, sdo_polling_thread, &nodes[i]);
    }

	while (!exit_flag) {
		UNS8 PDO_number = _retreive_PDO_queue_entry();
		if (PDO_number != NO_PDO_IN_QUEUE) {
			int result = sendOnePDOevent(&actuators_Data, PDO_number);
			printf("%d!\n", result);
		}
		usleep(50);
	}

    fprintf(stderr, "ACTUATORS EXITING\n");

	CANopen_shutdown();

	exit_flag = 1;

	pthread_join(ros_thread, NULL);
	pthread_join(matlab_thread, NULL);

	for (unsigned int i = 0; i < sizeof(nodes) / sizeof(nodes[0]); i++) {
		pthread_join(nodes[i].sdo_thread, NULL);
	}

	fclose(error_log);

	return result;
}
