/*
 * canopen-util.h
 *
 *  Created on: 27.06.2013
 *      Author: yury
 */

#ifndef CANOPEN_UTIL_H_
#define CANOPEN_UTIL_H_

#include <canfestival.h>
#include <data.h>

#include <queue>
#include <pthread.h>

void print_hex(char *data, int size);

UNS8 _getIndexSubindex(CO_Data *d, UNS8 nodeId, UNS16 *index, UNS8 *subindex);

#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

#define CALLBACK(INDEX, SUBINDEX) \
		void OD_index_ ## INDEX ##_ ## SUBINDEX (UNS8 *data, UNS32 size); \
			__attribute__ ((constructor))\
						void init_callback_ ## INDEX ## _ ## SUBINDEX() {\
								callbacks[callback_no].index = INDEX;	\
								callbacks[callback_no].subindex= SUBINDEX;\
								callbacks[callback_no].callback_fn = OD_index_ ## INDEX ##_ ## SUBINDEX; \
								callback_no++;\
						} \
		void OD_index_ ## INDEX ##_ ## SUBINDEX (UNS8 *data, UNS32 size)


#define PDO_CALLBACK(INDEX, COMMAND_NO, NAME) \
		UNS32 PDO_callback_ ## NAME ## _index_ ## INDEX ##_ ## COMMAND_NO (CO_Data *d, const indextable *unused, UNS8 Subindex); \
			__attribute__ ((constructor))\
						void init_pdo_callback_ ## INDEX () {\
								pdo_callbacks[pdo_callback_no].index = INDEX;	\
								pdo_callbacks[pdo_callback_no].callback_fn = PDO_callback_ ## NAME ## _index_ ## INDEX ##_ ## COMMAND_NO; \
								pdo_callback_no++;\
						} \
		UNS32 PDO_callback_ ## NAME ## _index_ ## INDEX ##_ ## COMMAND_NO (CO_Data *d, const indextable *unused, UNS8 Subindex)

#define DECLARE_PDO_CALLBACK_VARS \
	UNS8 data[1024]; \
	UNS8 dataType = octet_string; \
	UNS32 size = sizeof(data) / sizeof(data[0])

/* call this callback when SDO(index,subindex) is successfully finished */
struct Indigo_OD_Callback {
	UNS16 index;
	UNS8 subindex;

	void (*callback_fn)(UNS8 *data, UNS32 size);
};

struct PDO_callback {
	UNS16 index;

	UNS32 (*callback_fn)(CO_Data *d, const indextable *unused, UNS8 Subindex);
};


void my_sleep(int seconds);

extern std::queue<int> send_queue_COB;
extern pthread_mutex_t send_queue_COB_lock;
void enqueue_PDO(int PDO);
#define SEND_QUEUE_LOCK() do {\
	pthread_mutex_lock(&send_queue_COB_lock);\
} while(0)


#define SEND_QUEUE_UNLOCK() do {\
	pthread_mutex_unlock(&send_queue_COB_lock);\
} while(0)


#define SDO_USE_BLOCK_MODE 1
#define SDO_USE_EXPEDITED_MODE 0

#endif /* CANOPEN_UTIL_H_ */
