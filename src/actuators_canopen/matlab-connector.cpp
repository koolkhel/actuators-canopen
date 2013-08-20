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
#include "transport.h"
#include "matlab-connector.h"

int matlab_listen_socket = 0;
#define LISTEN_PORT 6100

pthread_mutex_t send_queue_lock = PTHREAD_MUTEX_INITIALIZER;
std::queue<std::string> send_queue;

int pipe_write_fd = 0;
int pipe_read_fd = 0;

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

extern struct message_sender *senders;

/**
 * Text protocol to communicate with matlab.
 * @param model model to be updated
 * @param line protocol message
 * @return 1 if anything was successfully parsed
 */
int proto_parse_line(struct actuators_model *model, const char *line)
{
	int result = 0;
	int tmp_int = 0;
	double tmp = 0.0;
	char tmp_char = 0;

	fprintf(stderr, "got line %s", line);

	if (result == 0) {
		result = sscanf(line, "start message %x", &tmp_int);
		if (result) {
			if ((tmp_int < 0x30) && (tmp_int > 0)) {
				senders[tmp_int].turned_off = 0;
			} else {
				fprintf(stderr, "invalid message id: %x", tmp_int);
			}
		}
	}

	if (result == 0) {
		result = sscanf(line, "stop message %x", &tmp_int);
		if (result) {
			if ((tmp_int < 0x30) && (tmp_int > 0)) {
				senders[tmp_int].turned_off = 1;
			} else {
				fprintf(stderr, "invalid message id: %x", tmp_int);
			}
		}
	}

	if (result == 0) {
		result = sscanf(line, "helium valve %c", &tmp_char);
		if (result) {
			model->left_ballonet_state.ballonet_valve_2 = (tmp_char == '1') ?
					COMMAND_BALLONET_FORCE_OPEN :COMMAND_BALLONET_FORCE_CLOSE;
		}
	}

	if (result == 0) {
		result = sscanf(line, "left engine rate %lf", &tmp);
		if (result)
			model->left_main_engine_rate = tmp;
	}

	if (result == 0)
		result = sscanf(line, "left engine angle %f", &model->left_main_engine_angle);

	if (result == 0) {
		result = sscanf(line, "right engine rate %lf", &tmp);
		if (result)
			model->right_main_engine_rate = tmp;
	}

	if (result == 0)
		result = sscanf(line, "right engine angle %f", &model->right_main_engine_angle);

	if (result == 0) {
		result = sscanf(line, "left electromotor rate %lf", &tmp);
		if (result)
			model->left_electromotor_rate = (WORD) tmp;
	}

	if (result == 0)
		result = sscanf(line, "left electromotor anglex %f", &model->left_electromotor_angle_X);

	if (result == 0)
		result = sscanf(line, "left electromotor angley %f", &model->left_electromotor_angle_Y);

	if (result == 0) {
		result = sscanf(line, "right electromotor rate %lf", &tmp);
		if (result)
			model->right_electromotor_rate = (WORD) tmp;
	}

	if (result == 0)
		result = sscanf(line, "right electromotor anglex %f", &model->right_electromotor_angle_X);

	if (result == 0)
		result = sscanf(line, "right electromotor angley %f", &model->right_electromotor_angle_Y);

	if (result == 0) {
		result = sscanf(line, "left ballonet fan1 %c", &tmp_char);
		if (result == 1) {
			if (tmp_char == '1') {
				model->left_ballonet_state.ballonet_fan_1 = COMMAND_BALLONET_FORCE_OPEN;
			} else if (tmp_char == '0') {
				model->left_ballonet_state.ballonet_fan_1 = COMMAND_BALLONET_FORCE_CLOSE;
			}
		}
	}

	if (result == 0) {
		result = sscanf(line, "left ballonet fan2 %c", &tmp_char);
		if (result == 1) {
			if (tmp_char == '1') {
				model->left_ballonet_state.ballonet_fan_2 = COMMAND_BALLONET_FORCE_OPEN;
			} else if (tmp_char == '0') {
				model->left_ballonet_state.ballonet_fan_2 = COMMAND_BALLONET_FORCE_CLOSE;
			}
		}
	}

	if (result == 0) { /* FIXME two valves */
		result = sscanf(line, "left ballonet valve %c", &tmp_char);
		if (result == 1) {
			if (tmp_char == '1') {
				model->left_ballonet_state.ballonet_valve_1 = COMMAND_BALLONET_FORCE_OPEN;
			} else if (tmp_char == '0') {
				model->left_ballonet_state.ballonet_valve_1 = COMMAND_BALLONET_FORCE_CLOSE;
			}
		}
	}

	if (result == 0) {
		result = sscanf(line, "right ballonet fan1 %c", &tmp_char);
		if (result == 1) {
			if (tmp_char == '1') {
				model->right_ballonet_state.ballonet_fan_1 = COMMAND_BALLONET_FORCE_OPEN;
			} else if (tmp_char == '0') {
				model->right_ballonet_state.ballonet_fan_1 = COMMAND_BALLONET_FORCE_CLOSE;
			}
		}
	}

	if (result == 0) {
		result = sscanf(line, "right ballonet fan2 %c", &tmp_char);
		if (result == 1) {
			if (tmp_char == '1') {
				model->right_ballonet_state.ballonet_fan_2 = COMMAND_BALLONET_FORCE_OPEN;
			} else if (tmp_char == '0') {
				model->right_ballonet_state.ballonet_fan_2 = COMMAND_BALLONET_FORCE_CLOSE;
			}
		}
	}

	if (result == 0) {
		result = sscanf(line, "right ballonet valve %c", &tmp_char);
		if (result == 1) {
			if (tmp_char == '1') {
				model->right_ballonet_state.ballonet_valve_1 = COMMAND_BALLONET_FORCE_OPEN;
			} else if (tmp_char == '0') {
				model->right_ballonet_state.ballonet_valve_1 = COMMAND_BALLONET_FORCE_CLOSE;
			}
		}
	}

	if (result == 0) { /* FIXME there was an errata about helium valve parameter location */
		result = sscanf(line, "ballonet helium valve %c", &tmp_char);
		if (result == 1) {
			if (tmp_char == '1') {
				model->right_ballonet_state.ballonet_valve_2 = COMMAND_BALLONET_FORCE_OPEN;
			} else if (tmp_char == '0') {
				model->right_ballonet_state.ballonet_valve_2 = COMMAND_BALLONET_FORCE_CLOSE;
			}
		}
	}


	if (result == 0) {
		result = sscanf(line, "left ballonet pressureDiff1 %lf", &tmp);
		if (result)
			model->left_ballonet_differential_pressure_1 = tmp;
	}

	if (result == 0) {
		result = sscanf(line, "left ballonet pressureDiff2 %lf", &tmp);
		if (result)
			model->left_ballonet_differential_pressure_2 = tmp;
	}

	if (result == 0) {
		result = sscanf(line, "right ballonet pressureDiff1 %lf", &tmp);
		if (result)
			model->right_ballonet_differential_pressure_1 = tmp;
	}

	if (result == 0) {
		result = sscanf(line, "right ballonet pressureDiff2 %lf", &tmp);
		if (result)
			 model->right_ballonet_differential_pressure_2 = tmp;
	}

	return result;
}

/**
 * Debug output model state
 * @param model
 */
void proto_dump_model(struct actuators_model	 *model) {

	printf("===========================\n");
	printf("left engine rate %hu\n", model->left_main_engine_rate);
	printf("left engine angle %f\n", model->left_main_engine_angle);
	printf("right engine rate %hu\n", model->right_main_engine_rate);
	printf("right engine angle %f\n", model->right_main_engine_angle);
	printf("left electromotor rate %hu\n", model->left_electromotor_rate);
	printf("left electromotor anglex %f\n", model->left_electromotor_angle_X);
	printf("left electromotor angley %f\n", model->left_electromotor_angle_Y);
	printf("right electromotor rate %hu\n", model->right_electromotor_rate);
	printf("right electromotor anglex %f\n", model->right_electromotor_angle_X);
	printf("right electromotor angley %f\n", model->right_electromotor_angle_Y);
	printf("left ballonet fan1 %s\n", model->left_ballonet_state.ballonet_fan_1 ? "true" : "false");
	printf("left ballonet fan2 %s\n", model->left_ballonet_state.ballonet_fan_2 ? "true" : "false");
	printf("left ballonet valve %s\n", model->right_ballonet_state.ballonet_valve_1 ? "true" : "false");
	printf("right ballonet fan1 %s\n", model->right_ballonet_state.ballonet_fan_1 ? "true" : "false");
	printf("right ballonet fan2 %s\n", model->right_ballonet_state.ballonet_fan_2 ? "true" : "false");
	printf("right ballonet valve %s\n", model->right_ballonet_state.ballonet_valve_1 ? "true" : "false");
	/* FIXME helium valve */
	printf("ballonet helium valve %s\n", model->right_ballonet_state.ballonet_valve_2 ? "true" : "false");

	printf("left ballonet pressureDiff1 %hu\n", model->left_ballonet_differential_pressure_1);
	printf("left ballonet pressureDiff2 %hu\n", model->left_ballonet_differential_pressure_2);
	printf("right ballonet pressureDiff1 %hu\n", model->right_ballonet_differential_pressure_1);
	printf("right ballonet pressureDiff2 %hu\n", model->right_ballonet_differential_pressure_2	);

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

	while (true) {
		/* FIXME write part */
		FD_ZERO(&read_fd_set);
		FD_SET(client_socket, &read_fd_set);
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
				proto_parse_line(&MODEL, buf);
				proto_dump_model(&MODEL);
				UNLOCK_MODEL();
			}

			if (FD_ISSET(pipe_read_fd, &read_fd_set)) {
				pthread_mutex_lock(&send_queue_lock);

				const char *send_buf = NULL;

				result = read(pipe_read_fd, &buf[0], 1);
				if (result <= 0) {
					perror("pipe read");
					break;
				}

				send_buf = send_queue.front().c_str();

				result = write(client_socket, send_buf, strlen(send_buf		));
				if (result <= 0) {
					perror("client socket write");
					break;
				}

				send_queue.pop();
				pthread_mutex_unlock(&send_queue_lock);
			}

		} else if (result == 0) { /* select timeout */

		} else {
			perror("matlab select");
		}
	}
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
