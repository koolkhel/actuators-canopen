#include <stdio.h>
#include <canfestival.h>
#include <data.h>
#include <sys/select.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>

#include <signal.h>

#include "./actuators.h"
#include "./model.h"
#include "./canopen-util.h"
#include "./canopen-data.h"
#include "./actuators-ros.h"

#include "./matlab-connector.h"

// #include <ros/ros.h>

#include <assert.h>

static char busname[] = "0";
static char baudrate[] = "500K";

s_BOARD Board = { "", "" };
FILE *error_log = NULL;

pthread_t ros_thread;
pthread_t expedited_sdo_thread;
pthread_t matlab_thread;

sig_atomic_t exit_flag = 0;

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

int callback_no = 0;

struct actuators_model MODEL;
extern pthread_mutex_t MODEL_lock;

struct Indigo_OD_Callback callbacks[200];

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
		fprintf(error_log, "%ld size mismatch: %d against reference %dm aborting\n", time(NULL), size, sizeof(struct data_## INDEX ## _## SUBINDEX));\
		printf("%ld size mismatch: %d against reference %d for index 0x" #INDEX " subindex 0x" #SUBINDEX "\n", time(NULL), size, sizeof(struct data_## INDEX ## _## SUBINDEX));\
	};\
	struct data_## INDEX ##_## SUBINDEX *M = (struct data_ ## INDEX ##_## SUBINDEX *) data

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

int pdo_callback_no = 0;

struct PDO_callback pdo_callbacks[512];

PDO_CALLBACK(0x5100, 0x188, left_motor_startstop_response) {
        DECLARE_PDO_CALLBACK_VARS;

        char new_data[8] = {0};
        unsigned int send_size = 0;

        readLocalDict(d, 0x5100, 0x0, data, &size, &dataType, 0);

        switch (Subindex) {
        case 0x0:
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

void notify_ros_topic(UNS16 index, UNS8 subindex) {
	int i = 0;

	for (i = 0; i < callback_no; i++) {
		if (callbacks[i].index == index && callbacks[i].subindex == subindex) {
			sem_post(&callbacks[i].semaphore);
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
	// остальное не читается, только задаётся
	// 0x200, 0x300, 0x400, 0x500 + 12

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
	// остальное не читается, только задаётся
	// 0x200, 0x300, 0x400, 0x500 + 13
	// right_ballonet_state.ballonet_fan_1
	// right_ballonet_state.ballonet_fan_2
	// right_ballonet_state.ballonet_valve_1

	// right_ballonet_differential_pressure_1
	// right_ballonet_differential_pressure_2
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

	while (!exit_flag) {
		for (UNS8 i = 0; i < pollable_entries_no; i++) {
			struct pollable_OD_entry *entry = &pollable_entries[i];

			if (entry->nodeId != nodeid)
				continue;

			if (entry->status == MATLAB_ENABLED)
				continue;

			UNS16 index = entry->index;
			UNS8 subindex = entry->subindex;
			UNS32 datatype = entry->datatype;
			UNS32 transfer_type = entry->transferType;

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

			while ((result == SDO_UPLOAD_IN_PROGRESS) || (result == SDO_BLOCK_UPLOAD_IN_PROGRESS)) {
				usleep(100);
				result = getReadResultNetworkDict(&actuators_Data, nodeid, &entry->data[0],
						&entry->size, &abort_code);
			}

			printf("result for SDO node 0x%hhx is %u\n", nodeid, result);

			if (result == SDO_FINISHED) {
				printf("SDO Result node 0x%hhx index 0x%hx subindex 0x%hhx: %d: ", nodeid, index, subindex, result);
				print_hex((char *) &entry->data[0], 8);
				printf("\n");
				CallSDOCallback(index, subindex, &entry->data[0], entry->size);
			} else {
				printf("SDO Result ABORT node 0x%hhx index 0x%hx subindex 0x%hhx code: %u\n", nodeid, index, subindex, abort_code);
				usleep(100 * 1000); // wait a little more for lower levels to handle this
			}
			closeSDOtransfer(&actuators_Data, nodeid, SDO_CLIENT);
		}
		usleep(100);
	}

	return NULL;
}

std::queue<int> send_queue_COB;
pthread_mutex_t send_queue_COB_lock;

pthread_mutex_t MODEL_lock;
pthread_mutex_t REPORTED_DATA_lock;

void BUS_CAN_HEARTBEAT_ERROR(CO_Data *data, UNS8 nodeId) {
	e_nodeState nodeState = getNodeState(data, nodeId);

	fprintf(stdout, "===================================\n");
	fprintf(stdout, "HEARTBEAT ERROR for NODE %hhu!!!!!!\n", nodeId);
	fprintf(stdout, "===================================\n");

	switch (nodeState) {
	case Initialisation:
		fprintf(stdout, "HEARTBEAT node %hhu state Initialisation\n", nodeId);
		break;
	case Disconnected:
		fprintf(stdout, "HEARTBEAT node %hhu state Disconnected\n", nodeId);
		break;
	case Preparing:
		fprintf(stdout, "HEARTBEAT node %hhu state Preparing/Connecting\n", nodeId);
		break;
	case Stopped:
		fprintf(stdout, "HEARTBEAT node %hhu state Stopped\n", nodeId);
		break;
	case Operational:
		fprintf(stdout, "HEARTBEAT node %hhu state Operational\n", nodeId);
		break;
	case Pre_operational:
		fprintf(stdout, "HEARTBEAT node %hhu state Pre_operational\n", nodeId);
		break;
	case Unknown_state:
		fprintf(stdout, "HEARTBEAT node %hhu state Unknown_state\n", nodeId);
		break;
	}
}

void post_sync(CO_Data *data) {
//	enqueue_PDO(0x19); // send control mode
	sendOnePDOevent(&actuators_Data, 0x19);
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

	actuators_Data.post_sync = post_sync;
	//actuators_Data.heartbeatError = BUS_CAN_HEARTBEAT_ERROR;
	//heartbeatInit(&actuators_Data);
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

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	pthread_mutex_init(&MODEL_lock, &attr);
	pthread_mutex_init(&REPORTED_DATA_lock, &attr);
	pthread_mutex_init(&send_queue_COB_lock, &attr);
}


/* ROS services use this for command mappings */
void enqueue_PDO(int PDO) {
	long int my_time = time(NULL);
	struct tm my_time_s;
	char buf[255];

	localtime_r(&my_time, &my_time_s);
	asctime_r(&my_time_s, buf);

	if (!MODEL.remote_control_enabled || (PDO == 0x19)) {
		SEND_QUEUE_LOCK();
		printf("%s QUEUEING PDO 0x%.02x...\n", buf, PDO);
		send_queue_COB.push(PDO);
		SEND_QUEUE_UNLOCK();
	} else {
		printf("%s NOT QUEUEING PDO 0x%.02x DUE TO REMOTE CONTROL\n", buf, PDO);
	}
}

int _retreive_PDO_queue_entry() {
	int PDO = -1;
	long int my_time = time(NULL);
	struct tm my_time_s;
	char buf[255];

	localtime_r(&my_time, &my_time_s);
	asctime_r(&my_time_s, buf);

	while (PDO == -1) {
		SEND_QUEUE_LOCK();
		if (!send_queue_COB.empty()) {
			PDO = send_queue_COB.front();
			printf("%s SENDING PDO 0x%.02x...", buf, PDO);
			send_queue_COB.pop();
			SEND_QUEUE_UNLOCK();
		} else {
			SEND_QUEUE_UNLOCK	();
			usleep(100);
		}
	}



	return PDO;
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

#undef DEBUG
int main(int argc, char **argv) {
	int result = 0;



	printf("starting actuators\n");
	printf("%d SDO callbacks\n", callback_no);
	for (int i = 0; i < callback_no; i++) {
		printf("%d SDO callback index 0x%hx subindex 0x%hhx address %p\n", i, callbacks[i].index, callbacks[i].subindex, callbacks[i].callback_fn);

		sem_init(&callbacks[i].semaphore, 0, 0);
	}

	printf("%d PDO callbacks\n", pdo_callback_no);
	for (int i = 0; i < pdo_callback_no; i++) {
		RegisterSetODentryCallBack(&actuators_Data, pdo_callbacks[i].index, 0, pdo_callbacks[i].callback_fn);
	}


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
		UNS8 data[1024];
		UNS32 size = 0;
		UNS32 abort_code = 0;

		/* TODO: drain send queue when appropriate -- maybe at start of SYNC period */

		SEND_QUEUE_LOCK();
		if (!send_queue_COB.empty()) {
			SEND_QUEUE_UNLOCK();
			int PDO_number = _retreive_PDO_queue_entry();
			int result = sendOnePDOevent(&actuators_Data, PDO_number);
			printf("%d!\n", result);
		} else {
			SEND_QUEUE_UNLOCK();
		}
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
