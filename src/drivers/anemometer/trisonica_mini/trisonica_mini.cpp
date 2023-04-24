/****************************************************************************
 *
 *   Copyright (c) 2014-2019, 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "trisonica_mini.hpp"

#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>

TrisonicaMini::TrisonicaMini(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* set device ID */
	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	/* serial bus number (assuming '/dev/ttySx') */
	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);
	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}
}

TrisonicaMini::~TrisonicaMini() {
	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TrisonicaMini::init() {
	start();
	return PX4_OK;
}

int TrisonicaMini::assemble(char c, char *parserbuf, unsigned *parserbuf_index, enum PARSE_STATE *state){
	TODO
}

int TrisonicaMini::parse(TODO){
	TODO
}

int TrisonicaMini::collect() {

	/* TODO: what is this and is it needed? */
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_packet)];
	unsigned readlen = sizeof(readbuf) - 1;

	/* read from the sensor (uart buffer) */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int bytes_read = ::read(_fd, &readbuf[0], readlen);
	if (bytes_read < 0) {
		PX4_DEBUG("read err: %d", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_interval * 2)) {
			return bytes_read;
		} else {
			return -EAGAIN;
		}
	} else if (bytes_read == 0) {
		/* zero bytes read without error. read again. */
		return -EAGAIN;
	}

	/* we have made a successful read. make note of the time and initialize. */
	_last_read = hrt_absolute_time();
	float <empty topic here> = <static const empty value>; // TODO:
	bool valid = false;

	/* TODO: Determine how we should parse the data based on the mode */
	// int32_t mode = 0;
	// param_get(param_find("SENS_EN_TRIMINI"), &mode);

	/* assemble a packet of raw bytes */
	for (size_t i = 0; i < bytes_read; i++) {
		if (OK == TrisonicaMini::assemble(readbuf[i], _packet, &_packet_index, &_parse_state)) {
			valid = true;
		}
	}
	if (!valid) {
		return -EAGAIN;
	}

	/* supposedly we have a complete packet we can now parse */
	int ret = TrisonicaMini::parse(_packet, &topic);

	// _px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

void TrisonicaMini::start() {
	ScheduleNow();
}

void TrisonicaMini::stop(){
	ScheduleClear();
}

void TrisonicaMini::Run()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, _baud)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}
		if ((termios_state = cfsetospeed(&uart_config, _baud)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}
		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void TrisonicaMini::print_info() {
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
