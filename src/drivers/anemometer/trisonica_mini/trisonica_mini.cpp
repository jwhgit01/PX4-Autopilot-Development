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

/* Configuration Constants */
#define LW_TAKE_RANGE_REG		'd'

TrisonicaMini::TrisonicaMini(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

}

TrisonicaMini::~TrisonicaMini()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

/* If we need to set the smapling interval based on a parameter, do it here. */
int TrisonicaMini::init()
{
	// This used to be in a switch statement based on the hardware model.
	// _interval = 50000;
	// _simple_serial = true; // For the rangefinder, this was an option for a different packet structure. We don't need it.

	start();

	return PX4_OK;
}

/* If we need to send a command to initiate a measurement, this is where we do that:
int TrisonicaMini::measure()
{
	// Send the command to begin a measurement.
	char cmd = LW_TAKE_RANGE_REG;
	int ret = ::write(_fd, &cmd, 1);

	if (ret != sizeof(cmd)) {
		perf_count(_comms_errors);
		PX4_DEBUG("write fail %d", ret);
		return ret;
	}

	return PX4_OK;
}
*/

int TrisonicaMini::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	/* read from the sensor (uart buffer) */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = ::read(_fd, &readbuf[0], readlen);
	if (ret < 0) {
		PX4_DEBUG("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_interval * 2)) {
			return ret;

		} else {
			return -EAGAIN;
		}

	} else if (ret == 0) {
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();
	
	/* create the topic to be published */
	sensor_anemometer_s report{};
	report.timestamp = _last_read;

	float vx_m_s, vy_m_s, vz_m_s, T_C;
	bool valid = false;
	
	/* loop through read buffer and parse data */
	for (int i = 0; i < ret; i++) {
		if (OK == trisonica_mini_parser(readbuf[i],
										_linebuf,
										&_linebuf_index,
										&_parse_state,
										&vx_m_s,
										&vy_m_s,
										&vz_m_s,
										&T_C)) {
			valid = true;
		}
	}


	if (!valid) {
		return -EAGAIN;
	}

	// TODO: Update this debugging.
	// PX4_DEBUG("val (float): %8.4f, raw: %s, valid: %s", (double)distance_m, _linebuf, ((valid) ? "OK" : "NO"));

	/* publish the sensor readings */
	_sensor_anemometer_pub.publish(report);

	perf_end(_sample_perf);

	return PX4_OK;
}

void TrisonicaMini::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void TrisonicaMini::stop()
{
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

		/* sensor baud rate */
		unsigned baud = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}

	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		// TODO: Update this timing if necessary
		if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
			ScheduleDelayed(1042 * 8);

			return;
		}

		if (OK != collect_ret) {

			// TODO: If the sensor needs time to initialize, do that here.
			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
				PX4_ERR("collection error #%u", _consecutive_fail_count);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			start();
			return;

		} else {
			/* apparently success */
			_consecutive_fail_count = 0;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void TrisonicaMini::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
