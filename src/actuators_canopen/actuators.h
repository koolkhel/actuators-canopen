
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 actuators_valueRangeTest (UNS8 typeValue, void * value);
const indextable * actuators_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);

/* Master node data struct */
extern CO_Data actuators_Data;
extern INTEGER64 LEFT_MOTOR_209;		/* Mapped at index 0x5000, subindex 0x00*/
extern INTEGER64 TAIL_ELECTROMOTOR_207;		/* Mapped at index 0x5001, subindex 0x00*/
extern INTEGER64 TAIL_ELECTROMOTOR_307;		/* Mapped at index 0x5002, subindex 0x00*/
extern INTEGER64 TAIL_ELECTROMOTOR_407;		/* Mapped at index 0x5003, subindex 0x00*/
extern INTEGER64 LEFT_BALLONET_20C;		/* Mapped at index 0x5004, subindex 0x00*/
extern INTEGER64 LEFT_BALLONET_30C;		/* Mapped at index 0x5005, subindex 0x00*/
extern INTEGER64 LEFT_BALLONET_40C;		/* Mapped at index 0x5006, subindex 0x00*/
extern INTEGER64 LEFT_BALLONET_50C;		/* Mapped at index 0x5007, subindex 0x00*/
extern INTEGER64 RIGHT_BALLONET_20D;		/* Mapped at index 0x5008, subindex 0x00*/
extern INTEGER64 RIGHT_BALLONET_30D;		/* Mapped at index 0x5009, subindex 0x00*/
extern INTEGER64 RIGHT_BALLONET_40D;		/* Mapped at index 0x500A, subindex 0x00*/
extern INTEGER64 RIGHT_BALLONET_50D;		/* Mapped at index 0x500B, subindex 0x00*/
extern INTEGER64 LEFT_MOTOR_ROTATION_30A;		/* Mapped at index 0x500C, subindex 0x00*/
extern INTEGER64 RIGHT_MOTOR_ROTATION_30B;		/* Mapped at index 0x500D, subindex 0x00*/
extern INTEGER64 LEFT_MOTOR_ROTATION_20A;		/* Mapped at index 0x500E, subindex 0x00*/
extern INTEGER64 RIGHT_MOTOR_ROTATION_20B;		/* Mapped at index 0x500F, subindex 0x00*/
extern INTEGER64 LEFT_MOTOR_208;		/* Mapped at index 0x5010, subindex 0x00*/
extern INTEGER64 RIGHT_MOTOR_209;		/* Mapped at index 0x5011, subindex 0x00*/
extern INTEGER64 LEFT_MOTOR_308;		/* Mapped at index 0x5012, subindex 0x00*/
extern INTEGER64 RIGHT_MOTOR_309;		/* Mapped at index 0x5013, subindex 0x00*/
extern INTEGER64 LEFT_MOTOR_312;		/* Mapped at index 0x5014, subindex 0x00*/
extern INTEGER64 RIGHT_MOTOR_313;		/* Mapped at index 0x5015, subindex 0x00*/
extern INTEGER64 POWER_DISTRIBUTION_RELAY_305;		/* Mapped at index 0x5016, subindex 0x00*/
extern INTEGER64 GENERATOR_205;		/* Mapped at index 0x5017, subindex 0x00*/
extern INTEGER64 HELIUM_VALVE_20e;		/* Mapped at index 0x5018, subindex 0x00*/
extern INTEGER64 REMOTE_CONTROL_ALLOW_201;		/* Mapped at index 0x5019, subindex 0x00*/
extern INTEGER64 CONTROL_SURFACE_206;		/* Mapped at index 0x501A, subindex 0x00*/
extern INTEGER8 LEFT_MOTOR_CALLBACK_188;		/* Mapped at index 0x5100, subindex 0x00*/
extern INTEGER8 RIGHT_MOTOR_CALLBACK_189;		/* Mapped at index 0x5101, subindex 0x00*/
extern INTEGER8 HELIUM_VALVE_CALLBACK_182;		/* Mapped at index 0x5102, subindex 0x00*/
extern INTEGER8 REMOTE_CONTROL_SWITCH_204;		/* Mapped at index 0x5103, subindex 0x00*/

#endif // ACTUATORS_H
