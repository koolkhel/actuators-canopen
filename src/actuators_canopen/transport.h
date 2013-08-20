#pragma once

#include <queue>
#include <stddef.h>
#include "actuators.h"

extern pthread_t can_thread;
void *can_main(void *data);

struct can_settings {
	int channel;
	int bitrate;
	/// mode to be replaced
	char mode;
};

struct CAN_receiving {
	/* true -- some data has been received, false -- empty */
	BYTE started;

	BYTE MID;
	BYTE MSX;
	union CAN_id next_id;

	BYTE next_frame_num;
	BYTE frame_count;
	WORD text_size;
	WORD read_text_size;

	/* malloc/free every packet */
	union CAN_logical_message message;
};

struct CAN_receivers {
	/* as many receivers as there are message kinds */
	struct CAN_receiving receiver[CAN_MESSAGE_MAX_ID];
};

#define THIS_RECEIVER RECEIVER.receiver[message->MID]

/**
 * information needed to perform a given MID message sending
 */
struct message_sender {
	int MID;
	void (*send)(union CAN_logical_message *message, uint8_t *frame_count, uint16_t *text_size);

	/* everything static goes to the table, dynamic goes to special function */
	enum CAN_priority priority;

	enum CAN_node_address source;
	enum CAN_node_address destination;

	BYTE MSX;

	BYTE turned_off;
};

struct message_receiver {
		int MID;
		void (*receive)(union CAN_logical_message *);
};

/// construct union CAN_logical_message property name upon given MID
#define MSG(NUMBER) message->message_ ## NUMBER

/**
 * fill given parameter in outgoing message struct from MODEL global variable
 * @param NUMBER
 * @param PARAMETER
 */
#define SEND(NUMBER, PARAMETER) MSG(NUMBER).PARAMETER = MODEL.PARAMETER; fprintf(stderr, "sending " STRINGIFY(PARAMETER) "\n")
#define SEND_DEBUG(NUMBER, PARAMETER, FORMAT) do {\
	MSG(NUMBER).PARAMETER = MODEL.PARAMETER; \
	fprintf(stdout, "sending " STRINGIFY(PARAMETER) ": " FORMAT " with raw bytes: ", MODEL.PARAMETER);\
	int offset = offsetof(CAN_message_ ## NUMBER, PARAMETER);\
	print_hex_bytes(((unsigned char *)&MSG(NUMBER)) + offset, sizeof(MSG(NUMBER).PARAMETER));\
	fprintf(stdout, "\n");\
} while(0)

#define SEND_VAL(NUMBER, PARAMETER, VAL) (MSG(NUMBER).PARAMETER = VAL)

#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)
/**
 * fill corresponding REPORTED_DATA property upon known message ID and data from received CAN message
 * @param NUMBER
 * @param PARAMETER
 */
#define RECEIVE(NUMBER, PARAMETER) do {\
	REPORTED_DATA.PARAMETER = MSG(NUMBER).PARAMETER; \
	fprintf(stderr,"receiving " STRINGIFY(PARAMETER) "\n");\
} while (0)

#define RECEIVE_DEBUG(NUMBER, PARAMETER, FORMAT) do {\
	REPORTED_DATA.PARAMETER = MSG(NUMBER).PARAMETER; \
	fprintf(stdout,"receiving " STRINGIFY(PARAMETER) ": " FORMAT "", REPORTED_DATA.PARAMETER);\
	fprintf(stdout, " with raw bytes : "); \
	int offset = offsetof(CAN_message_ ## NUMBER, PARAMETER);\
	print_hex_bytes(((unsigned char *)&MSG(NUMBER)) + offset, sizeof(MSG(NUMBER).PARAMETER));\
	fprintf(stdout, "\n");\
} while (0)

/// message body = (frame_count, (text_size, data)), text_size == full_message_size - 1
#define FRAME_COUNT_BYTE_BIAS 1

/// fill frame_count property for known message number
#define FRAME_COUNT(NUMBER, FC) do { \
		MSG(NUMBER).frame_count = FC; \
		*frame_count = FC; \
		fprintf(stderr, "frame count == %d\n", FC);\
	} while(0)
/// fill text_size of multi-framed message, 16 bit unsigned, byte-swapping required
/// full message size is returned to SENDER function argument ( + FRAME_COUNT_BYTE_BIAS)
#define TEXT_SIZE_WORD(NUMBER, TS) do { \
		MSG(NUMBER).text_size = FLIP_ENDIAN_WORD((WORD) TS); \
		*text_size = (WORD) TS + FRAME_COUNT_BYTE_BIAS; \
		fprintf(stderr, "text size word is %d\n", TS);\
	} while (0)
/// fill text_size of single message
#define TEXT_SIZE_BYTE(NUMBER, TS) do { \
		MSG(NUMBER).text_size = TS; \
		*text_size = TS + 1;\
		assert(*text_size <= 8);\
		fprintf(stderr, "text size byte is %d\n", TS);\
	} while(0)
/// get text_size of multi-framed message
#define GET_TEXT_SIZE_WORD(NUMBER) (FLIP_ENDIAN_WORD((WORD) MSG(NUMBER).text_size))
/// get text_size of single_message
#define GET_TEXT_SIZE_BYTE(NUMBER) (MSG(NUMBER).text_size)
/// get frame count
#define GET_FRAME_COUNT(NUMBER) (MSG(NUMBER).frame_count)

/// construct sending function prototype for given message ID
#define SENDER(NUM, NAME, SOURCE, DESTINATION) \
	void send_ ## NAME ##_ ## NUM (union CAN_logical_message *message, uint8_t *frame_count, uint16_t *text_size); \
		__attribute__ ((constructor))\
					void init_sender_ ## NAME ## _ ## NUM() {\
							SENDERS_ ## NAME [0x ## NUM].MID = 0x ## NUM;	\
							SENDERS_ ## NAME [0x ## NUM].send = send_ ## NAME ##_ ## NUM; \
							SENDERS_ ## NAME [0x ## NUM].source = SOURCE; \
							SENDERS_ ## NAME [0x ## NUM].destination = DESTINATION; \
					} \
	void send_ ## NAME ##_ ## NUM (union CAN_logical_message *message, uint8_t *frame_count, uint16_t *text_size)

/// common sending function initialization for arguments not to get out uninitialized
/// also mutex lock
#define SENDER_START() do {\
	*frame_count = 0; \
	*text_size = 0; \
	printf("sender start %s\n", __func__); \
	LOCK_MODEL();\
} while(0)

#define SENDER_FINISH() do {\
	printf("sender stop %s\n", __func__); \
	UNLOCK_MODEL();\
} while(0);

/// construct receiving function prototype for given message ID
#define RECEIVER(NUM, NAME) \
		void receive_ ## NAME ##_ ## NUM (union CAN_logical_message *message);\
		__attribute__ ((constructor))\
				void init_receiver_ ## NAME ## _ ## NUM() {\
						RECEIVERS_ ## NAME [0x ## NUM].MID = 0x ## NUM;	\
						RECEIVERS_ ## NAME [0x ## NUM].receive = receive_ ## NAME ##_ ## NUM; \
				} \
		void receive_ ## NAME ##_ ## NUM (union CAN_logical_message *message)


/// similiar pair for mutexed access
#define RECEIVER_START() do {\
	LOCK_REPORTED();\
} while (0)

#define RECEIVER_FINISH() do {\
	UNLOCK_REPORTED(); \
} while (0)

extern std::queue<int> send_queue_MID;
extern pthread_mutex_t send_queue_MID_lock;
void enqueue_MID(int MID);
#define SEND_QUEUE_LOCK() do {\
	pthread_mutex_lock(&send_queue_MID_lock);\
} while(0)
#define SEND_QUEUE_UNLOCK() do {\
	pthread_mutex_unlock(&send_queue_MID_lock);\
} while(0)

void process_transport_level(struct CAN_message *message);
void print_CAN_message_debug(struct CAN_message *message);
char process_raw_CAN_message(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag, struct CAN_message *message);
void handle_CAN_message(long id, unsigned char msg[8], unsigned int dlc, unsigned int flag);

