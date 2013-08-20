/*
 * actuators-ros.h
 *
 *  Created on: 15.12.2012
 *      Author: yury
 */

#pragma once

#include <pthread.h>

extern pthread_t ros_thread;

struct actuators_ros_settings {
	int argc;
	char **argv;
};

void *ros_main(void *data);
