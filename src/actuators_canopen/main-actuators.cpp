#include <stdio.h>
#include <canfestival.h>
#include <data.h>
#include <sys/select.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>

#include <signal.h>

#include "./actuators.h"
#include "./model.h"
#include "./canopen-util.h"
#include "./canopen-data.h"
#include "./actuators-ros.h"

#include <assert.h>

static char busname[] = "0";
static char baudrate[] = "500K";

s_BOARD Board = { "", "" };
FILE *error_log = NULL;

pthread_t ros_thread;

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
		fprintf(error_log, "%ld size mismatch: %d against reference %d\n", time(NULL), size, sizeof(struct data_## INDEX ## _## SUBINDEX));\
		printf("%ld size mismatch: %d against reference %d\n", time(NULL), size, sizeof(struct data_## INDEX ## _## SUBINDEX));\
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

PDO_CALLBACK(0x5101, 0x189, right_motor_startstop_response) {
        DECLARE_PDO_CALLBACK_VARS;

        static unsigned char new_data[8] = {0};
        unsigned int send_size = 0;

        readLocalDict(d, 0x5101, 0x0, data, &size, &dataType, 0);

        switch (Subindex) {
        case 0x0:
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
                break;
        default:
                printf("unknown PDO subindex %.02hhX\n", Subindex);
                break;
        }

        return OD_SUCCESSFUL;
}

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
	(void)M;
}

// PAYLOAD_BLOCK
CALLBACK(0x4004, 0x10) {

}

CALLBACK(0x4004, 0x20) {
	DECLARE_M(4004, 20);
	(void) M;
}

// POWER
CALLBACK(0x4005, 0x10) {

}

CALLBACK(0x4005, 0x20) {

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

}

// TAIL_ENGINE
CALLBACK(0x4007, 0x10) {

}


CALLBACK(0x4007, 0x20) {
	DECLARE_M(4007, 20);

	RECEIVE_PRINT(M->left_electromotor_angle_X, left_electromotor_angle_X, 100.0, "%f");
	RECEIVE_PRINT(M->left_electromotor_angle_Y, left_electromotor_angle_Y, 100.0, "%f");
	RECEIVE_PRINT(M->left_electromotor_rate, left_electromotor_rate, 1, "%d");

	RECEIVE_PRINT(M->right_electromotor_angle_X, right_electromotor_angle_X, 100.0, "%f");
	RECEIVE_PRINT(M->right_electromotor_angle_Y, right_electromotor_angle_Y, 100.0, "%f");
	RECEIVE_PRINT(M->right_electromotor_rate, right_electromotor_rate, 1, "%d");
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
}

// LEFT MOTOR
CALLBACK(0x4008, 0x10) {

}

CALLBACK(0x4008, 0x20) {
	DECLARE_M(4008, 20);

	// 1 rpm
	RECEIVE_PRINT(M->left_main_engine_rate, left_main_engine_rate, 1, "%hu RPM");

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

	MODEL.left_main_engine_relay_state = M->left_main_engine_relay_state;
}

// RIGHT MOTOR
CALLBACK(0x4009, 0x10) {
}

CALLBACK(0x4009, 0x20) {
	DECLARE_M(4009, 20);

	RECEIVE_PRINT(M->right_main_engine_rate, right_main_engine_rate, 1, "%hu RPM");

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

	MODEL.right_main_engine_relay_state = M->right_main_engine_relay_state;
}

// LEFT_MOTOR_ROTATE
CALLBACK(0x400a, 0x10) {

}

CALLBACK(0x400a, 0x20) {
	DECLARE_M(400a, 20);

	RECEIVE_PRINT(M->left_main_engine_rotation_angle, left_main_engine_rotation_angle, 100.0, "%f degrees");
}

CALLBACK(0x400a, 0x21) {
	DECLARE_M(400a, 21);

	RECEIVE_PRINT(M->left_main_engine_rotation_current, left_main_engine_rotation_current, 10.0, "%f A");
	RECEIVE_PRINT(M->left_main_engine_rotation_voltage, left_main_engine_rotation_voltage, 10.0, "%f V");
}

// RIGHT_MOTOR_ROTATE
CALLBACK(0x400b, 0x20) {
	DECLARE_M(400b, 20);

	RECEIVE_PRINT(M->right_main_engine_rotation_angle, right_main_engine_rotation_angle, 100.0, "%f degrees");
}

CALLBACK(0x400b, 0x21) {
	DECLARE_M(400b, 21);

	RECEIVE_PRINT(M->right_main_engine_rotation_current, right_main_engine_rotation_current, 10.0, "%f A");
	RECEIVE_PRINT(M->right_main_engine_rotation_voltage, right_main_engine_rotation_voltage, 10.0, "%f V");
}



CALLBACK(0x400c, 0x10) {

};

CALLBACK(0x400c, 0x20) {
	// остальное не читается, только задаётся
	// 0x200, 0x300, 0x400, 0x500 + 12

	DECLARE_M(400c, 20);

	RECEIVE_PRINT(M->left_ballonet_pressure_1, left_ballonet_pressure_1, 1, "%hu Pa");
	RECEIVE_PRINT(M->left_ballonet_pressure_2, left_ballonet_pressure_2, 1, "%hu Pa");
	RECEIVE_PRINT(M->left_ballonet_lights_state, left_ballonet_lights_state, 1, "%hu state");
	RECEIVE_PRINT(M->left_ballonet_control, left_ballonet_control, 1, "%hu");
	RECEIVE_PRINT(M->left_ballonet_linear_valve_resistance, left_ballonet_linear_valve_resistance, 1, "%f ???");
};

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

	RECEIVE_PRINT(M->right_ballonet_pressure_1, right_ballonet_pressure_1, 1, "%hu Pa");
	RECEIVE_PRINT(M->right_ballonet_pressure_2, right_ballonet_pressure_2, 1, "%hu Pa");
	RECEIVE_PRINT(M->right_ballonet_lights_state, right_ballonet_lights_state, 1, "%hu state");
	RECEIVE_PRINT(M->right_ballonet_control, right_ballonet_control, 1, "%hu");
	RECEIVE_PRINT(M->right_ballonet_linear_valve_resistance, right_ballonet_linear_valve_resistance, 1, "%f ???");
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


    // 1 == in progress, in order to abort
    // 0 == reset
    int status;
};

struct pollable_OD_entry pollable_entries[] = {
    //	{0x05, 0x4005, 0x21, octet_string, INDIGO_SDO_BLOCK}, // power system
    //	{0x07, 0x4007, 0x20, octet_string, INDIGO_SDO_BLOCK}, // tail electromotor
        {0x08, 0x4008, 0x20, octet_string, INDIGO_SDO_BLOCK}, // left main engine
    //	{0x09, 0x4009, 0x20, octet_string, INDIGO_SDO_BLOCK}, // right main engine
    //	{0x0a, 0x400a, 0x20, octet_string, INDIGO_SDO_BLOCK}, // left rotation
    //	{0x0a, 0x400a, 0x21, octet_string, INDIGO_SDO_BLOCK}, // left rotation
    //	{0x0b, 0x400b, 0x20, octet_string, INDIGO_SDO_EXPEDITED}, // right rotation
    //	{0x0b, 0x400b, 0x21, octet_string, INDIGO_SDO_EXPEDITED} // right rotation
    //	{0x0c, 0x400c, 0x20, octet_string, INDIGO_SDO_BLOCK}, // ballonet left
    //	{0x0d, 0x400d, 0x20, octet_string, INDIGO_SDO_BLOCK} // ballonet right
};

int findPollableODEntry(UNS16 index, UNS8 subindex) {
    int size = sizeof(pollable_entries) / sizeof(pollable_entries[0]);
    bool found = false;
    int i = -1;

    for (i = 0; i < size; i++) {
        if (pollable_entries[i].index == index && pollable_entries[i].subindex == subindex) {
            found = true;
            break;
        }
    }
    return found ? i : -1;
}

void CheckReadSDO(CO_Data *d, UNS8 nodeid);
void restartSDO(UNS8 nodeid, UNS16 index, UNS8 subindex);

void restartSDObyNode(UNS8 nodeid) {
    TRACE_ENTRY();

    printf("restarting (multiple) SDO for node %hhu\n", nodeid);

    for (int i = 0; i < sizeof(pollable_entries) / sizeof(pollable_entries[0]); i++) {
        if (pollable_entries[i].nodeId == nodeid) {
            if (pollable_entries[i].status == 1) {
                pollable_entries[i].status = 0;
                printf("restarting SDO index %d\n", i);
                restartSDO(nodeid, pollable_entries[i].index, pollable_entries[i].subindex);
            }
        }
    }
    TRACE_EXIT();
}

void restartSDO(UNS8 nodeid, UNS16 index, UNS8 subindex) {
    int entry_index = 0;
    int result = 0;
    UNS32 abort_code = 0;

    TRACE_ENTRY();

    entry_index = findPollableODEntry(index, subindex);

    if (entry_index == -1) {
        printf("couldn't find pollable entry for index %hu subindex %hhu, assuming abort case\n", index, subindex);
        restartSDObyNode(nodeid);

    }  else {

        UNS8 datatype = pollable_entries[entry_index].datatype;
        UNS8 transferType = pollable_entries[entry_index].transferType;

        switch (transferType) {
        case INDIGO_SDO_BLOCK:
            // block transfer works in async

            printf("sending SDO %hhu %hu %hhu\n", nodeid, index, subindex);

            pollable_entries[entry_index].status = 1;
            result = readNetworkDictCallback(&actuators_Data, nodeid,
                                             index, subindex, datatype, CheckReadSDO, SDO_USE_BLOCK_MODE);
            break;

        case INDIGO_SDO_EXPEDITED:
            // expedited transfer works in sync now
            result = readNetworkDict(&actuators_Data, nodeid,
                                     index, subindex, datatype, SDO_USE_EXPEDITED_MODE);

            pollable_entries[entry_index].size = 1024;
            memset(&pollable_entries[entry_index].data[0], 0, pollable_entries[entry_index].size);

            pollable_entries[entry_index].status = 1;
            result = getReadResultNetworkDict(&actuators_Data, nodeid, &pollable_entries[entry_index].data[0],
                                              &pollable_entries[entry_index].size, &abort_code);
            // printf("\nResult %d: ", result);
            while (result == SDO_UPLOAD_IN_PROGRESS) {
                usleep(1000);
                // printf("\nResult %d: ", result);
                result = getReadResultNetworkDict(&actuators_Data, nodeid, &pollable_entries[entry_index].data[0],
                                                  &pollable_entries[entry_index].size, &abort_code);
            }

            if (result == SDO_FINISHED) {
                printf("\nResult %d: ", result);
                print_hex((char *) &pollable_entries[entry_index].data[0], 8);
                printf("\n");
                CallSDOCallback(index, subindex, &pollable_entries[entry_index].data[0],
                                pollable_entries[entry_index].size);
            }
            pollable_entries[entry_index].status = 0;
            break;

        default:
            break;
        }
    }

    TRACE_EXIT();
}

/** Callback function that check the read SDO demand.
 *
 */
void CheckReadSDO(CO_Data *d, UNS8 nodeid) {
	UNS32 abortCode;
	UNS8 data[128] = { 0 };
    UNS32 size = sizeof(data);

	UNS16 index = 0;
	UNS8 subindex = 0;
	UNS8 err = 0;

    printf("CheckReadSDO called!\n");

	err = _getIndexSubindex(d, nodeid, &index, &subindex);
	if (err) {
		fprintf(error_log, "%ld couldn't find getIndexSubindex for nodeid 0x%hhx index 0x%hx\n", time(NULL), nodeid, index);
	}

	if (getReadResultNetworkDict(d, nodeid, &data, &size,
            &abortCode) != SDO_FINISHED || err)
		printf(
				"\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n",
				nodeid, abortCode);
    else if (!err) {
		printf("\nResult : ");
		print_hex((char *) data, size);
		printf("\n");
		CallSDOCallback(index, subindex, data, size);

        int entry_index = findPollableODEntry(index, subindex);
        if (entry_index != -1) {
            pollable_entries[entry_index].status = 0;
        }
	}

out:
	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(d, nodeid, SDO_CLIENT);

    //printf("calling restartSDO index %hu subindex %hhu\n", index, subindex);
    restartSDO(nodeid, index, subindex);
}

std::queue<int> send_queue_COB;
pthread_mutex_t send_queue_COB_lock;

pthread_mutex_t MODEL_lock;
pthread_mutex_t REPORTED_DATA_lock;

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
    setNodeId(&actuators_Data, 0x1);

	setState(&actuators_Data, Operational);
}

void CANopen_shutdown(void) {
	// Stop timer thread
	StopTimerLoop(&Exit);

	canClose(&actuators_Data);

	TimerCleanup();
}

/**
 * Возвращает индекс (0x1800 + x) для PDO по его номеру
 * @param COB
 * @return
 */
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


/* ROS services use this for command mappings */
void enqueue_PDO(int PDO) {
	long int my_time = time(NULL);
	struct tm my_time_s;
	char buf[255];

	localtime_r(&my_time, &my_time_s);
	asctime_r(&my_time_s, buf);

	SEND_QUEUE_LOCK();
	printf("%s QUEUEING PDO 0x%.02x...\n", buf, PDO);
	send_queue_COB.push(PDO);
	SEND_QUEUE_UNLOCK();
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

sig_atomic_t exit_flag = 0;

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

	CANopen_startup();

	init_model();

	start_ros_thread(argc, argv);


		// SDO
		// III
    my_sleep(1);

    for (unsigned int i = 0; i < sizeof(pollable_entries) / sizeof(pollable_entries[0]); i++) {
        UNS8 nodeId = pollable_entries[i].nodeId;
        UNS16 index = pollable_entries[i].index;
        UNS8 subindex = pollable_entries[i].subindex;


        printf("calling RESTART_SDO\n");
        restartSDO(nodeId, index, subindex);

        usleep(1000);
    }

	while (!exit_flag) {
		UNS8 data[1024];
		UNS32 size = 0;
		UNS32 abort_code = 0;

		/* TODO: drain send queue when appropriate -- maybe at start of SYNC period */
		int PDO_number = _retreive_PDO_queue_entry();

		int result = sendOnePDOevent(&actuators_Data, PDO_number);
		printf("%d!\n", result);
	}


	CANopen_shutdown();

	exit_flag = 1;

	pthread_join(ros_thread, NULL);

	fclose(error_log);

	return result;
}
