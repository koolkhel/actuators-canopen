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

extern pthread_mutex_t send_queue_lock;
extern std::queue<std::string> send_queue;
extern int pipe_write_fd;
extern int pipe_read_fd;

#define QUEUE_MATLAB_NOTIFY(VAR, FORMAT) do {\
	char buf[255];\
	int result;\
	snprintf(buf, 255, FORMAT "\n", REPORTED_DATA.VAR);\
	pthread_mutex_lock(&send_queue_lock);\
	send_queue.push(buf);\
	fprintf(stderr, "pushing to matlab: %s\n", buf);\
	result = write(pipe_write_fd, &buf[0], 1); /* signal to select that a message has come */ \
	if (!result) perror("matlab write");\
	pthread_mutex_unlock(&send_queue_lock);\
} while(0)

void *matlab_main(void *data);
