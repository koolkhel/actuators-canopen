/*
 * matlab-connector.cpp
 *
 *  Created on: 15.12.2012
 *      Author: yury
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>

#include <canfestival.h>
#include <sys/select.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>

#include <queue>
#include <string>


#include "model.h"
#include "matlab-connector.h"

int matlab_listen_socket = 0;
#define LISTEN_PORT 6100

int pipe_write_fd = 0;
int pipe_read_fd = 0;

volatile int matlab_connected  = 0;

/**
 * read from descriptor until \n is reached
 * @param fd
 * @param buffer
 * @param n
 * @return how many bytes are read
 */
ssize_t readLine(int fd, char *buffer, size_t n)
{
    ssize_t numRead;                    /* # of bytes fetched by last read() */
    size_t totRead;                     /* Total bytes read so far */
    char *buf;
    char ch;

    if (n <= 0 || buffer == NULL) {
        errno = EINVAL;
        return -1;
    }

    buf = buffer;                       /* No pointer arithmetic on "void *" */

    totRead = 0;
    for (;;) {
        numRead = read(fd, &ch, 1);

        if (numRead == -1) {
            if (errno == EINTR || errno == EAGAIN)         /* Interrupted --> restart read() */
                continue;
            else
                return -1;              /* Some other error */

        } else if (numRead == 0) {      /* EOF */
            if (totRead == 0)           /* No bytes read; return 0 */
                return 0;
            else                        /* Some bytes read; add '\0' */
                break;

        } else {                        /* 'numRead' must be 1 if we get here */
            if (totRead < n - 1) {      /* Discard > (n - 1) bytes */
                totRead++;
                *buf++ = ch;
            }

            if (ch == '\n')
                break;
        }
    }

    *buf = '\0';
    return totRead;
}

void notify_ros_topic(UNS16 index, UNS8 subindex);
void reset_matlab_feedback();
void enable_matlab_feedback(UNS8 nodeid);

/**
 * Text protocol to communicate with matlab.
 * @param model model to be updated
 * @param line protocol message
 * @return 1 if anything was successfully parsed
 */
int proto_parse_line(struct actuators_model *model, const char *line)
{
	int result = 0;

	fprintf(stderr, "MATLAB got line %s", line);

	if (result == 0) {
		result = sscanf(line, "left_electromotor_rate %hu", &model->left_electromotor_rate);
		if (result) {
			notify_ros_topic(0x4007, 0x20);
			enable_matlab_feedback(0x7);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_electromotor_rate %hu", &model->right_electromotor_rate);
		if (result) {
			notify_ros_topic(0x4007, 0x20);
			enable_matlab_feedback(0x7);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_electromotor_angle_Y %f", &model->left_electromotor_angle_Y);
		if (result) {
			notify_ros_topic(0x4007, 0x20);
			enable_matlab_feedback(0x7);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_electromotor_angle_Y %f", &model->right_electromotor_angle_Y);
		if (result) {
			notify_ros_topic(0x4007, 0x20);
			enable_matlab_feedback(0x7);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_electromotor_angle_X %f", &model->left_electromotor_angle_X);
		if (result) {
			notify_ros_topic(0x4007, 0x20);
			enable_matlab_feedback(0x7);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_electromotor_angle_X %f", &model->right_electromotor_angle_X);
		if (result) {
			notify_ros_topic(0x4007, 0x20);
			enable_matlab_feedback(0x7);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_main_engine_rate %hu", &model->left_main_engine_rate);
		if (result) {
			notify_ros_topic(0x4008, 0x20);
			enable_matlab_feedback(0x8);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_main_engine_fuel_level %f", &model->left_main_engine_fuel_level);
		if (result) {
			notify_ros_topic(0x4008, 0x20);
			enable_matlab_feedback(0x8);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_main_engine_aux_fuel_level %f", &model->left_main_engine_aux_fuel_level);
		if (result) {
			notify_ros_topic(0x4000 + 0x8, 0x20);
			enable_matlab_feedback(0x8);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_main_engine_throttle_1_angle %f", &model->left_main_engine_throttle_1_angle);
		if (result) {
			notify_ros_topic(0x4000 + 0x8, 0x20);
			enable_matlab_feedback(0x8);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_main_engine_throttle_2_angle %f", &model->left_main_engine_throttle_2_angle);
		if (result) {
			notify_ros_topic(0x4000 + 0x8, 0x20);
			enable_matlab_feedback(0x8);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_main_engine_rate %hu", &model->right_main_engine_rate);
		if (result) {
			notify_ros_topic(0x4009, 0x20);
			enable_matlab_feedback(0x9);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_main_engine_fuel_level %f", &model->right_main_engine_fuel_level);
		if (result) {
			notify_ros_topic(0x4000 + 0x9, 0x20);
			enable_matlab_feedback(0x9);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_main_engine_aux_fuel_level %f", &model->right_main_engine_aux_fuel_level);
		notify_ros_topic(0x4009, 0x20);
		enable_matlab_feedback(0x9);
	}
	if (result == 0) {
		result = sscanf(line, "right_main_engine_throttle_1_angle %f", &model->right_main_engine_throttle_1_angle);
		if (result) {
			notify_ros_topic(0x4009, 0x20);
			enable_matlab_feedback(0x9);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_main_engine_throttle_2_angle %f", &model->right_main_engine_throttle_2_angle);
		if (result) {
			notify_ros_topic(0x4009, 0x20);
			enable_matlab_feedback(0x9);
		}
	}
	if (result == 0) {
		result = sscanf(line, "left_main_engine_rotation_angle %f", &model->left_main_engine_rotation_angle);
		if (result) {
			notify_ros_topic(0x400a, 0x20);
			enable_matlab_feedback(0xa);
		}
	}
	if (result == 0) {
		result = sscanf(line, "right_main_engine_rotation_angle %f", &model->right_main_engine_rotation_angle);
		if (result) {
			notify_ros_topic(0x400b, 0x20);
			enable_matlab_feedback(0xb);
		}
	}

	// 0.01 degree
	if (result == 0) {
		result = sscanf(line, "left_control_surface_X %f", &model->left_control_surface_X);
		if (result) {
			notify_ros_topic(0x4006, 0x20);
			enable_matlab_feedback(0x6);
		}
	}

	if (result == 0) {
		result = sscanf(line, "left_control_surface_Y %f", &model->left_control_surface_Y);
		if (result) {
			notify_ros_topic(0x4006, 0x20);
			enable_matlab_feedback(0x6);
		}
	}

	if (result == 0) {
		result = sscanf(line, "right_control_surface_X %f", &model->right_control_surface_X);
		if (result) {
			notify_ros_topic(0x4006, 0x20);
			enable_matlab_feedback(0x6);
		}
	}

	if (result == 0) {
		result = sscanf(line, "right_control_surface_Y %f", &model->right_control_surface_Y);
		if (result) {
			notify_ros_topic(0x4006, 0x20);
			enable_matlab_feedback(0x6);
		}
	}

	return result;
}

/**
 * Debug output model state
 * @param model
 */
void proto_dump_model(struct actuators_model *model) {

	printf("===========================\n");
	printf("left engine rate %hu\n",         model->left_main_engine_rate);
	printf("left engine angle %f\n",         model->left_main_engine_rotation_angle);
	printf("right engine rate %hu\n",        model->right_main_engine_rate);
	printf("right engine angle %f\n",        model->right_main_engine_rotation_angle);
	printf("left electromotor rate %hu\n",   model->left_electromotor_rate);
	printf("left electromotor anglex %f\n",  model->left_electromotor_angle_X);
	printf("left electromotor angley %f\n",  model->left_electromotor_angle_Y);
	printf("right electromotor rate %hu\n",  model->right_electromotor_rate);
	printf("right electromotor anglex %f\n", model->right_electromotor_angle_X);
	printf("right electromotor angley %f\n", model->right_electromotor_angle_Y);
	printf("===========================\n");

	return;
}

/*
 * Perform read/write
 *
 */
void server_loop(int client_socket) {
	struct timeval tv;
	fd_set read_fd_set, write_fd_set;
	int result = 0;

#define MATLAB_BUF_SIZE 1024

	char buf[MATLAB_BUF_SIZE];
	char log_buf[MATLAB_BUF_SIZE];

	int max_fd = (client_socket > pipe_read_fd) ?
			client_socket : pipe_read_fd;

	matlab_connected = 1;

	static struct timeval matlab_timeout_tv = {0, 0};
	gettimeofday(&matlab_timeout_tv, NULL);

	while (true) {
		/* FIXME write part */
		FD_ZERO(&read_fd_set);
		FD_ZERO(&write_fd_set);

		// matlab client
		FD_SET(client_socket, &read_fd_set);
		FD_SET(client_socket, &write_fd_set);

		// internal pipe for outgoing updates for matlab
		FD_SET(pipe_read_fd, &read_fd_set);

		tv.tv_sec = 3;
		tv.tv_usec = 0;

		struct timeval now_tv;
		long long diff = 0;

		gettimeofday(&now_tv, NULL);

		diff = now_tv.tv_sec * 1000 * 1000L + now_tv.tv_usec - (matlab_timeout_tv.tv_sec * 1000 * 1000L + matlab_timeout_tv.tv_usec);

		// more than 3 seconds

		if (diff > 3 * 1000 * 1000L) // matlab timeout
			break;


		result = select(max_fd + 1, &read_fd_set, &write_fd_set, NULL, &tv);
		if (result > 0) {
			if (FD_ISSET(client_socket, &read_fd_set)) {
				result = readLine(client_socket, buf, MATLAB_BUF_SIZE);
				if (result <= 0) {
					strerror_r(errno, log_buf, MATLAB_BUF_SIZE);
					fprintf(stderr, "matlab client READ error: %s\n", log_buf);
					break;
				}

				// parse incoming line from matlab
				proto_parse_line(&MODEL, buf);

				gettimeofday(&matlab_timeout_tv, NULL);
			}

			if (FD_ISSET(pipe_read_fd, &read_fd_set) && FD_ISSET(client_socket, &write_fd_set)) {

				char send_buf[256] = {0};

				result = readLine(pipe_read_fd, send_buf, sizeof(send_buf));
				if (result <= 0) {
					strerror_r(errno, log_buf, MATLAB_BUF_SIZE);
					fprintf(stderr, "matlab client PIPE error: %s\n", log_buf);
				}

				fprintf(stderr, "sending back to MATLAB %s", send_buf);

				result = write(client_socket, send_buf, strlen(send_buf));
				if (result <= 0) {
					strerror_r(errno, log_buf, MATLAB_BUF_SIZE);
					fprintf(stderr, "matlab client WRITE error: %s, exiting\n", log_buf);
					//break;
				}
			}

		} else if (result == 0) { /* select timeout */
			break; // exit as per Roman request
		} else {
			strerror_r(errno, log_buf, MATLAB_BUF_SIZE);
			fprintf(stderr, "matlab client SELECT error: %s, exiting\n", log_buf);
			break;
		}
	}

	matlab_connected = 0;
	reset_matlab_feedback();
}

extern sig_atomic_t exit_flag;

void *matlab_main(void *data) {
	int result = 0;
	int optval = 1;
	struct sockaddr_in my_addr;
	int pipe_fds[2];

	result = pipe2(pipe_fds, O_NONBLOCK);
	if (result) {
		perror("pipe");
		// more obvious way of telling we've got a problem
		exit(1);
		//pthread_exit(NULL);
	}

	pipe_read_fd = pipe_fds[0];
	pipe_write_fd = pipe_fds[1];

	while (!exit_flag) {
		if (matlab_listen_socket != 0)
			close(matlab_listen_socket);

		matlab_listen_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (matlab_listen_socket < 0) {
			perror("socket");
			sleep(1);
			continue;
		}

		setsockopt(matlab_listen_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
		optval = 1;
		setsockopt(matlab_listen_socket, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));

		memset(&my_addr, 0, sizeof(my_addr));

		my_addr.sin_family = AF_INET;
		my_addr.sin_port = htons(LISTEN_PORT);
		my_addr.sin_addr.s_addr = INADDR_ANY;

		result = bind(matlab_listen_socket, (const sockaddr *) &my_addr, sizeof(my_addr));
		if (result) {
			perror("bind");
			close(matlab_listen_socket);
			sleep(1);
			continue;
		}

		result = listen(matlab_listen_socket, 16);
		if (result) {
			perror("listen");
			close(matlab_listen_socket);
			sleep(1);
			continue;
		}

		while (true) {
			struct sockaddr_in client_addr;
			unsigned int client_addr_len;
			memset(&client_addr, 0, sizeof(client_addr));
			int client_socket = -1;
			client_addr_len = sizeof(client_addr);

			client_socket = accept(matlab_listen_socket, (sockaddr *) &client_addr, &client_addr_len);
			if (client_socket == -1) {
				perror("accept");
				sleep(1);
				break;
			}

			fprintf(stderr, "MATLAB client connect\n");

			/* presumably never exits */
			server_loop(client_socket);

			result = shutdown(client_socket, SHUT_RDWR);

			close(client_socket);

			fprintf(stderr, "MATLAB client disconnect\n");
		} // accept loop
	}

	return NULL;
}
