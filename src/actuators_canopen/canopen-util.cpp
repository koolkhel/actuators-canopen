/*
 * canopen-util.c
 *
 *  Created on: 27.06.2013
 *      Author: yury
 */

#include <sys/select.h>
#include <unistd.h>

#include "canopen-util.h"

void print_hex(char *data, int size) {
	int i = 0;
	for (i = 0; i < size; i++) {
		printf("0x%02hhX ", data[i]);
	}
}

void my_sleep(int seconds) {
	struct timeval timeout;

	timeout.tv_sec = seconds;
	timeout.tv_usec = 0;

	select(0, NULL, NULL, NULL, &timeout);
}


extern "C" {
UNS8 GetSDOClientFromNodeId(CO_Data* d, UNS8 nodeId);
}

UNS8 _getIndexSubindex(CO_Data *d, UNS8 nodeId, UNS16 *index, UNS8 *subindex) {
	UNS8 CliNbr;
	UNS8 line;
	UNS8 err = 0;

	/* First let's find the corresponding SDO client in our OD  */
	CliNbr = GetSDOClientFromNodeId(d, nodeId);
	if (CliNbr >= 0xFE) {
		printf("couldn't find client\n");
		err = 1;
		goto out;
		//index = 0;
		//*index = 5; // to fall with a core dump
	}

	/* Looking for the line tranfert. */
	err = getSDOlineOnUse(d, CliNbr, SDO_CLIENT, &line);
	if (err) {
		printf("couldn't find SDO line in use, aborting\n");
		//index = 0;
		//*index = 5; // >:-[
	} else {
		*index = d->transfers[line].index;
		*subindex = d->transfers[line].subIndex;
		//	printf("found index %04hX subindex %02hhx\n", *index, *subindex);
	}

out:
	return err;
}
