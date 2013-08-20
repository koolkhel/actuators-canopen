/*
 * emulator.cpp
 *
 *  Created on: 15.12.2012
 *      Author: yury
 */

#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <canlib.h>

#include "transport.h"
#include "matlab-connector.h"

pthread_t can_thread;
pthread_t matlab_thread;
pthread_t emulator_thread;

void start_can_thread(int argc, char **argv) {
	int result;

	static struct can_settings settings;

	if (argc < 2) {
		fprintf(stderr, "please provide channel number\n");
		exit(1);
	}

	settings.bitrate = BAUD_1M;
	settings.channel = atoi(argv[1]);
	settings.mode = 'e'; /* emulator */

	/* we want SIGINT to be passed to can thread */
	sigset_t set;
	sigaddset(&set, SIGINT);
	result = pthread_sigmask(SIG_BLOCK, &set, NULL);
	if (result) {
		perror("pthread_sigmask");
		exit(1);
	}

	if ((result = pthread_create(&can_thread, NULL, can_main, &settings))) {
		fprintf(stderr, "could'nt create can thread: %s %d\n", strerror(result), result);
		exit(1);
	}
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
		fprintf(stderr, "could'nt create matlab thread: %s %d\n", strerror(result), result);
		exit(1);
	}
}

void *emulator_main(void *args) {
	ros::Time::init();
	ros::Rate ten_hz(10);

	while (true) {
		enqueue_MID(0x08);
		enqueue_MID(0x0c);
		enqueue_MID(0x10);
		enqueue_MID(0x16);
		enqueue_MID(0x1c);
		enqueue_MID(0x20);
		enqueue_MID(0x24);

		if (!ten_hz.sleep())
			break;
	}

	return NULL;
}

void start_emulator_thread(int argc, char **argv) {
	int result = 0;
	if ((result = pthread_create(&emulator_thread, NULL, emulator_main, NULL))) {
		fprintf(stderr, "could'nt create can thread: %s %d\n", strerror(result), result);
		exit(1);
	}
}

int main(int argc, char **argv) {
	printf("emulators start\n");

	start_can_thread(argc, argv);
	start_matlab_thread(argc, argv);
	start_emulator_thread(argc, argv);

	pthread_join(can_thread, NULL);
	pthread_cancel(matlab_thread);
	pthread_cancel(emulator_thread);

	return 0;
}
