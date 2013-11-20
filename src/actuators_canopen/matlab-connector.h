/*
 * matlab-connector.h
 *
 *  Created on: 15.12.2012
 *      Author: yury
 */

#pragma once

#include <pthread.h>
#include <string>
#include <queue>

extern pthread_mutex_t matlab_send_queue_lock;
extern std::queue<std::string> matlab_send_queue;
extern int pipe_write_fd;
extern int pipe_read_fd;
extern volatile int matlab_connected;

#define QUEUE_MATLAB_NOTIFY(VAR, FORMAT) do {\
	char buf[255];\
	int result = 0;\
	if (matlab_connected) { \
		snprintf(buf, 255, FORMAT "\n", MODEL.VAR);\
		pthread_mutex_lock(&matlab_send_queue_lock);\
		matlab_send_queue.push(buf);\
		fprintf(stderr, "pushing to matlab: %s\n", buf);\
		result = write(pipe_write_fd, &buf[0], 1); /* signal to select that a message has come */ \
		if (!result) \
			perror("matlab write");\
		pthread_mutex_unlock(&matlab_send_queue_lock);\
	} else { \
	}\
} while(0)

#define QUEUE_MATLAB_NOTIFY_VAR(VAR, FORMAT) do {\
	char buf[255];\
	int result = 0;\
	if (matlab_connected) { \
		snprintf(buf, 255, FORMAT "\n", VAR);\
		pthread_mutex_lock(&matlab_send_queue_lock);\
		matlab_send_queue.push(buf);\
		fprintf(stderr, "pushing to matlab: %s\n", buf);\
		result = write(pipe_write_fd, &buf[0], 1); /* signal to select that a message has come */ \
		if (!result) \
			perror("matlab write");\
		pthread_mutex_unlock(&matlab_send_queue_lock);\
	} else { \
	}\
} while(0)

void *matlab_main(void *data);
