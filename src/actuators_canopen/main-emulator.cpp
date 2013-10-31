#include <stdio.h>
#include <canfestival.h>
#include <unistd.h>

#include "emulator.h"
#include "canopen-util.h"

static char busname[] = "1";
static char baudrate[] = "500K";
s_BOARD Board = { "", "" };

void Init(CO_Data* d, UNS32 id) {
	if (Board.baudrate) {
		/* Init node state*/
		setState(&emulator_Data, Initialisation);
	}
}

/***************************  CLEANUP  *****************************************/
void Exit(CO_Data* d, UNS32 nodeid) {
	if (strcmp(Board.baudrate, "none")) {
		/* Reset all nodes on the network */
		//masterSendNMTstateChange(&emulator_Data, 0 , NMT_Reset_Node);
		/* Stop master */
		setState(&emulator_Data, Stopped);
	}
}

/* Callback function that check the read SDO demand */
void CheckReadSDO(CO_Data* d, UNS8 nodeid) {
	UNS32 abortCode;
	UNS32 data = 0;
	UNS32 size = 64;

	if (getReadResultNetworkDict(d, nodeid, &data, &size,
			&abortCode) != SDO_FINISHED)
		printf(
				"\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n",
				nodeid, abortCode);
	else
		printf("\nResult : %x\n", data);

	/* Finalize last SDO transfer with this node */
	closeSDOtransfer(d, nodeid, SDO_CLIENT);
}

int callback_no = 0;

struct PDO_callback callbacks[512];

PDO_CALLBACK(0x5000, 0x209, wtf) {
	DECLARE_PDO_CALLBACK_VARS;

	readLocalDict(d, 0x5000, 0x0, data, &size, &dataType, 0);

	switch (Subindex) {
	case 0x0:
		printf("PARSED callback result 209: ");
		print_hex((char *) data, size);
		printf("\n");
		break;
	default:
		printf("unknown PDO subindex %.02hhX\n", Subindex);
		break;
	}

	return OD_SUCCESSFUL;
}

int main(int argc, char **argv) {
	int i = 0;
	Board.busname = busname;
	Board.baudrate = baudrate;

	LoadCanDriver("/usr/local/lib/libcanfestival_can_kvaser.so");

	setNodeId(&emulator_Data, 0x7);

	for (i = 0; i < callback_no; i++) {
		RegisterSetODentryCallBack(&emulator_Data, callbacks[i].index, 0, callbacks[i].callback_fn);
	}

	/* Init stack timer */
	TimerInit();

	canOpen(&Board, &emulator_Data);

	/* Start Timer thread */
	StartTimerLoop(&Init);

	my_sleep(1);

	setState(&emulator_Data, Pre_operational);

	my_sleep(1);

	setState(&emulator_Data, Operational);

	my_sleep(1);

	pause();

	// Stop timer thread
	StopTimerLoop(&Exit);

	canClose(&emulator_Data);

	TimerCleanup();

	return 0;
}
