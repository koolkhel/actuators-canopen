/*
 * matlab-connector.cpp
 *
 *  Created on: 15.12.2012
 *      Author: yury
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <queue>
#include <string>

#include "model.h"
#include "matlab-connector.h"

int matlab_listen_socket = 0;
#define LISTEN_PORT 6100

pthread_mutex_t matlab_send_queue_lock = PTHREAD_MUTEX_INITIALIZER;
std::queue<std::string> matlab_send_queue;

int pipe_write_fd = 0;
int pipe_read_fd = 0;

volatile int matlab_connected = 0;

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
            if (errno == EINTR)         /* Interrupted --> restart read() */
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

	if (result == 0) { result = sscanf(line, "left_electromotor_rate %hu", &model->left_electromotor_rate);notify_ros_topic(0x4007, 0x20);enable_matlab_feedback(0x7);}
	if (result == 0) { result = sscanf(line, "right_electromotor_rate %hu", &model->right_electromotor_rate);notify_ros_topic(0x4007, 0x20);enable_matlab_feedback(0x7);}
	if (result == 0) { result = sscanf(line, "left_electromotor_angle_Y %f", &model->left_electromotor_angle_Y);notify_ros_topic(0x4007, 0x20);enable_matlab_feedback(0x7);}
	if (result == 0) { result = sscanf(line, "right_electromotor_angle_Y %f", &model->right_electromotor_angle_Y);notify_ros_topic(0x4007, 0x20);enable_matlab_feedback(0x7);}
	if (result == 0) { result = sscanf(line, "left_electromotor_angle_X %f", &model->left_electromotor_angle_X);notify_ros_topic(0x4007, 0x20);enable_matlab_feedback(0x7);}
	if (result == 0) { result = sscanf(line, "right_electromotor_angle_X %f", &model->right_electromotor_angle_X);notify_ros_topic(0x4007, 0x20);enable_matlab_feedback(0x7);}
	if (result == 0) { result = sscanf(line, "left_main_engine_rate %hu", &model->left_main_engine_rate);notify_ros_topic(0x4008, 0x20);enable_matlab_feedback(0x8);}
	if (result == 0) { result = sscanf(line, "left_main_engine_fuel_level %f", &model->left_main_engine_fuel_level);notify_ros_topic(0x4008, 0x20);enable_matlab_feedback(0x8);}
	if (result == 0) { result = sscanf(line, "left_main_engine_aux_fuel_level %f", &model->left_main_engine_aux_fuel_level);notify_ros_topic(0x4008, 0x20);enable_matlab_feedback(0x8);}
	if (result == 0) { result = sscanf(line, "left_main_engine_throttle_1_angle %f", &model->left_main_engine_throttle_1_angle);notify_ros_topic(0x4008, 0x20);enable_matlab_feedback(0x8);}
	if (result == 0) { result = sscanf(line, "left_main_engine_throttle_2_angle %f", &model->left_main_engine_throttle_2_angle);notify_ros_topic(0x4008, 0x20);enable_matlab_feedback(0x8);}
	if (result == 0) { result = sscanf(line, "right_main_engine_rate %hu", &model->right_main_engine_rate);notify_ros_topic(0x4009, 0x20);enable_matlab_feedback(0x9);}
	if (result == 0) { result = sscanf(line, "right_main_engine_fuel_level %f", &model->right_main_engine_fuel_level);notify_ros_topic(0x4009, 0x20);enable_matlab_feedback(0x9);}
	if (result == 0) { result = sscanf(line, "right_main_engine_aux_fuel_level %f", &model->right_main_engine_aux_fuel_level);notify_ros_topic(0x4009, 0x20);enable_matlab_feedback(0x9);}
	if (result == 0) { result = sscanf(line, "right_main_engine_throttle_1_angle %f", &model->right_main_engine_throttle_1_angle);notify_ros_topic(0x4009, 0x20);enable_matlab_feedback(0x9);}
	if (result == 0) { result = sscanf(line, "right_main_engine_throttle_2_angle %f", &model->right_main_engine_throttle_2_angle);notify_ros_topic(0x4009, 0x20);enable_matlab_feedback(0x9);}
	if (result == 0) { result = sscanf(line, "left_main_engine_rotation_angle %f", &model->left_main_engine_rotation_angle);notify_ros_topic(0x400a, 0x20);enable_matlab_feedback(0x9);}
	if (result == 0) { result = sscanf(line, "right_main_engine_rotation_angle %f", &model->right_main_engine_rotation_angle);notify_ros_topic(0x400b, 0x20);enable_matlab_feedback(0x9);}

	return result;
}

/**
 * Debug output model state
 * @param model
 */
void proto_dump_model(struct actuators_model *model) {

	printf("===========================\n");
	printf("left engine rate %hu\n", model->left_main_engine_rate);
	printf("left engine angle %f\n", model->left_main_engine_rotation_angle);
	printf("right engine rate %hu\n", model->right_main_engine_rate);
	printf("right engine angle %f\n", model->right_main_engine_rotation_angle);
	printf("left electromotor rate %hu\n", model->left_electromotor_rate);
	printf("left electromotor anglex %f\n", model->left_electromotor_angle_X);
	printf("left electromotor angley %f\n", model->left_electromotor_angle_Y);
	printf("right electromotor rate %hu\n", model->right_electromotor_rate);
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
	fd_set read_fd_set;
	int result = 0;
	char buf[255];

	int max_fd = (client_socket > pipe_read_fd) ? client_socket : pipe_read_fd;

	matlab_connected = 1;

	while (true) {
		/* FIXME write part */
		FD_ZERO(&read_fd_set);
		// matlab client
		FD_SET(client_socket, &read_fd_set);
		// internal pipe for outgoing updates for matlab
		FD_SET(pipe_read_fd, &read_fd_set);

		tv.tv_sec = 1;
		tv.tv_usec = 0;

		result = select(max_fd + 1, &read_fd_set, NULL, NULL, &tv);
		if (result > 0) {
			if (FD_ISSET(client_socket, &read_fd_set)) {
				result = readLine(client_socket, buf, 255);
				if (result <= 0) {
					perror("EOF on client socket");
					break;
				}

				LOCK_MODEL();
				// parse incoming line from matlab
				proto_parse_line(&MODEL, buf);
				UNLOCK_MODEL();
			}

			if (FD_ISSET(pipe_read_fd, &read_fd_set)) {
				pthread_mutex_lock(&matlab_send_queue_lock);

				const char *send_buf = NULL;

				result = read(pipe_read_fd, &buf[0], 1);
				if (result <= 0) {
					perror("pipe read");
					break;
				}

				send_buf = matlab_send_queue.front().c_str();

				result = write(client_socket, send_buf, strlen(send_buf));
				if (result <= 0) {
					perror("client socket write");
					break;
				}

				matlab_send_queue.pop();
				pthread_mutex_unlock(&matlab_send_queue_lock);
			}

		} else if (result == 0) { /* select timeout */

		} else {
			perror("matlab select");
		}
	}

	matlab_connected = 0;
	reset_matlab_feedback();
}

void *matlab_main(void *data) {
	int result = 0;
	int optval = 1;
	struct sockaddr_in my_addr;
	int pipe_fds[2];

	result = pipe(pipe_fds);
	if (result) {
		perror("pipe");
		pthread_exit(NULL);
	}

	pipe_read_fd = pipe_fds[0];
	pipe_write_fd = pipe_fds[1];

	matlab_listen_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (matlab_listen_socket < 0) {
		perror("socket");
		pthread_exit(NULL);
	}

	setsockopt(matlab_listen_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

	memset(&my_addr, 0, sizeof(my_addr));

	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(LISTEN_PORT);
	my_addr.sin_addr.s_addr = INADDR_ANY;

	result = bind(matlab_listen_socket, (const sockaddr *) &my_addr, sizeof(my_addr));
	if (result) {
		perror("bind");
		pthread_exit(NULL);
	}

	result = listen(matlab_listen_socket, 2);
	if (result) {
		perror("listen");
		pthread_exit(NULL);
	}

	while (true) {
		struct sockaddr_in client_addr;
		unsigned int client_addr_len;
		memset(&client_addr, 0, sizeof(client_addr));
		int client_socket = 0;

		client_socket = accept(matlab_listen_socket, (sockaddr *) &client_addr, &client_addr_len);
		if (client_socket == -1) {
			perror("accept");
			continue;
		}

		/* presumably never exits */
		server_loop(client_socket);

		result = shutdown(client_socket, SHUT_RDWR);

		close(client_socket);
	}

	return NULL;
}
