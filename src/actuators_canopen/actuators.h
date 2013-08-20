
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

#endif // ACTUATORS_H