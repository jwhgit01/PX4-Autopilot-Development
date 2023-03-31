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

#include "TrisonicaMini.hpp"
#include <lib/drivers/device/Device.hpp>


TrisonicaMini::TrisonicaMini(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// device::Device::DeviceId device_id;
	// device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	// uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	// if (bus_num < 10) {
	// 	device_id.devid_s.bus = bus_num;
	// }


}

TrisonicaMini::~TrisonicaMini()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TrisonicaMini::init()
{
	start();

	return PX4_OK;
}

int TrisonicaMini::collect()
{
	perf_begin(_sample_perf);

	/* create variables to be populated by the parser */

	double V_m_s, direction_deg, u_m_s, v_m_s, w_m_s, T_C;
	bool valid = false;

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* read from the sensor (uart buffer) */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	char readbuf[sizeof(_buffer)];
	int bytes_read = ::read(_file_descriptor, &readbuf[0], sizeof(readbuf));

	/* handle read errors */
	if (bytes_read < 0) {
		PX4_INFO("read err: %d", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		if (read_elapsed > (_interval * 2)) {
			return bytes_read;
		} else {
			return -EAGAIN;
		}
	} else if (bytes_read == 0) {
		usleep(10);
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();
	// for(int i = 0; i < bytes_read; i++){
	// 	PX4_INFO("%s", &readbuf[i]);
	// }
	// PX4_INFO("\n");

	/* loop through read buffer and parse data */
	// PX4_INFO("parsingthethings:\n");
	PX4_INFO("\n");

		for (int i = 0; i < bytes_read; i++) {
			if (1 == trisonica_mini_parser(readbuf[i],_buffer,&_buffer_index,&_parse_state,&V_m_s,&direction_deg,&u_m_s,&v_m_s,&w_m_s,&T_C)) {
				valid = true;
			}
		}

		if (!valid) {
			return -EAGAIN;
		}

	PX4_INFO("VMS:%f\n", V_m_s);


	/* create and polulate the topic to be published */
	sensor_anemometer_s report{};
	report.timestamp = timestamp_sample;
	report.v_in_m_s = V_m_s;
	report.direction_deg = direction_deg;
	report.u_m_s = u_m_s;
	report.v_m_s = v_m_s;
	report.w_m_s = w_m_s;
	report.t_c = T_C;

	/* publish the sensor readings */
	_sensor_anemometer_pub.publish(report);

	perf_end(_sample_perf);

	return PX4_OK;
}

int TrisonicaMini::open_serial_port(const speed_t speed)
{
	/* file descriptor initialized? */
	if (_file_descriptor > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	/* configure port flags for read/write, non-controlling, non-blocking. */
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	/* open the serial port. */
	_file_descriptor = ::open(_port, flags);

	/* check for errors */
	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}
	if (!isatty(_file_descriptor)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	struct termios uart_config;
	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_file_descriptor, &uart_config);
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	/* Clear ONLCR flag (which appends a CR for every LF). */
	uart_config.c_oflag &= ~ONLCR;

	/* No parity, one stop bit. */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* No line processing - echo off, echo newline off, canonical mode off, extended input processing off, signal chars off */
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* set baud rate and check for errors*/
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}
	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}
	if ((termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void TrisonicaMini::Run()
{
	/* make sure the serial port is open */
	open_serial_port();

	/* perform collection */
	int ret_col = collect();
	if(ret_col < 0){
		PX4_INFO("RETCOL:%d\n", ret_col);
	}

	/* Possible TODO: do something based on ret_col */
}

void TrisonicaMini::start()
{
	/* schedule the driver at regular intervals */
	ScheduleOnInterval(READ_INTERVAL);
}

void TrisonicaMini::stop()
{
	/* Ensure the serial port is closed. */
	::close(_file_descriptor);

	/* Clear the work queue schedule. */
	ScheduleClear();
}

void TrisonicaMini::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
