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

#include "spingarage.hpp"

Spingarage::Spingarage(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* Port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* Serial bus number (/dev/ttySx) and set device ID */
	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);
	if (bus_num < 10) {
		_device_ID = (int) bus_num;
	} else {
		PX4_ERR("Invalid device ID from port number!\n");
	}
}

Spingarage::~Spingarage() {
	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Spingarage::init() {
	start();
	return PX4_OK;
}

int Spingarage::parse(const char* packet_data) {
	/* Read and assign values to the data */
	float V_kn, betaf_deg, alpha_deg, pitot_mbar, static_mbar, h_ft, alpha_mV, betaf_mV;
	int scan_result = sscanf(packet_data,
	        "$ADC,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
	        &V_kn,
		&h_ft,
		&pitot_mbar,
		&static_mbar,
		&alpha_deg,
		&betaf_deg,
		&alpha_mV,
		&betaf_mV
	);

	/* Check the scan result. If something is wrong, try again. */
	if (scan_result != 8) {
		PX4_ERR("Invalid string format from spingarage: %d tokens read", scan_result);
		return PX4_ERROR;
	}

	/* If we have made it here, we have a good packet of data */
	PX4_DEBUG("%s", packet_data);

	/* Units conversion */
	float V_m_s = V_kn*KN2MS;
	float betaf_rad = betaf_deg*M_DEG_TO_RAD_F;
	float alpha_rad = alpha_deg*M_DEG_TO_RAD_F;
	float pitot_Pa = pitot_mbar*MBAR2PA;
	float static_Pa = static_mbar*MBAR2PA;
	float h_m = h_ft*FT2M;

	/* Create and assign values to the sensor_airdata message */
	sensor_airdata_s sensor_airdata{};
	sensor_airdata.timestamp = timestamp_us;
	sensor_airdata.device_id = _device_ID;
	sensor_airdata.airspeed_m_s = V_m_s;
	sensor_airdata.flank_angle_rad = betaf_rad;
	sensor_airdata.angle_of_attack_rad = alpha_rad;
	sensor_airdata.pitot_pressure_pa = pitot_Pa;
	sensor_airdata.static_pressure_pa = static_Pa;
	sensor_airdata.pressure_alt_m = h_m;
	_sensor_airdata_pub.publish(sensor_airdata);

	/* Successfully read and published a packet of data! Keep reading! */
	return PX4_OK;
}

int Spingarage::collect() {
	/* We are begining a sample */
	PX4_DEBUG("Starting collection...\n");
	perf_begin(_sample_perf);

	/* Make note of the time since the last read */
	hrt_abstime read_elapsed = hrt_elapsed_time(&_last_read);

	/* Create the read buffer */
	char readbuf[_readlen];

	/* Perform a read */
	int bytes_read = ::read(_fd, &readbuf[0], _readlen);
	PX4_DEBUG("Bytes read: %d\n", bytes_read);

	/* Check for read errors */
	if (bytes_read == 0) {
		PX4_DEBUG("Zero bytes read without error. Read again.\n");
		return -EAGAIN;
	} else if (bytes_read < 0) {
		PX4_ERR("Read error: %d", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* Throw an error if we don't have a read for 5 seconds */
		if (read_elapsed > 5*1_s) {
			PX4_ERR("Something is wrong! Exiting out.");
			return PX4_ERROR;
		}

		/* Try again */
		return -EAGAIN;
	}

	/* If we made it here, we have a successful read form the serial port */
	_last_read = hrt_absolute_time();

	/* Loop through the read buffer and assemble a packet */
	for (int i = 0; i < bytes_read; i++) {

		/* If we haven't already found it, look for the start of the packet character */
		if (!_assemble_packet && readbuf[i] == _starting_char) {
			_packet_idx = 0; // Start a new packet
			_packet[_packet_idx] = readbuf[i]; // Store the start of packet character
			timestamp_us = hrt_absolute_time(); // Get the timestamp in microseconds
			_assemble_packet = true; // Start assembling the rest of the packet
			PX4_DEBUG("Start of packet character found!"); //debug
		}

		/* If we have found a start of packet, continue assembling data.
		 * If we have not found a start of packet nor are assembling one, move along. */
		else if (_assemble_packet) {
			/* Copy data into the packet */
			_packet[_packet_idx] = readbuf[i];

			/* If we hit an end of packet character, terminate the string and parse the packet. */
			if (readbuf[i] == _ending_char) {
				_packet[_packet_idx] = '\0'; // Null terminate the packet
				_assemble_packet = false; // Stop assembling packet
				int parse_result = parse(_packet); // Parse the data and publish it!
				if (parse_result != 0) {
					PX4_ERR("Packet parsing failed. Moving on.");
				}
			}

			/* If we receive another start character, something is wrong */
			else if (readbuf[i] == _starting_char) {
				_packet_idx = 0; // Start over
				_packet[_packet_idx] = readbuf[i]; // Put the new start of packet character into the packet
				timestamp_us = hrt_absolute_time(); // Get the data timestamp in microseconds
			}

			/* If we hit a null ('\0') character, something is wrong. Reset everything. */
			else if	(readbuf[i] == '\0') {
				_assemble_packet = 0;
				PX4_ERR("Something is wrong, read again!");
				return -EAGAIN; // Read again
			}

			/* If the packet index is too large, handle buffer overrun */
			else if (_packet_idx > PACKETLEN - 1) {
				PX4_ERR("Buffer overrun!\n");
				_overrun_count++; // Increase the counter
				_packet[_packet_idx] = '\0'; // Terminate with a null
				_packet_idx = -1; // Set the packet index to -1 as it will get incremented later
				_assemble_packet = false; // Look for start of packet character
				if (_overrun_count > 10) {
					_overrun_count = 0; // reset the count
					PX4_ERR("Exiting out, something is wrong!\n");
					return PX4_ERROR;
				}
				PX4_ERR("Trying again!\n");
				return -EAGAIN; // Read again
			}
		}

		/* If we have made it here, we need to continue assembling the packet.
		 * Increment the packet index. */
		_packet_idx++;
	}

	/* No more data to read! (end of read buffer reached) */
	perf_end(_sample_perf);
	return PX4_OK;
}

int Spingarage::open_serial_port() {
	/* file descriptor initialized? */
	if (_fd > 0) {
		PX4_DEBUG("serial port already open");
		return PX4_OK;
	}

	/* configure port flags for read/write, non-controlling, non-blocking. */
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	/* open the serial port. */
	_fd = ::open(_port, flags);

	/* check for errors */
	if (_fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}
	if (!isatty(_fd)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	struct termios uart_config;
	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;	/* 8-bit characters */
	uart_config.c_cflag &= ~PARENB;	/* no parity bit */
	uart_config.c_cflag &= ~CSTOPB;	/* only need 1 stop bit */
	uart_config.c_iflag &= ~ICRNL; 	/* no CR to NL translation */
	uart_config.c_iflag |= IGNPAR; 	/* Ignore parity errors */

	/* no flow control */
	uart_config.c_cflag &= ~CRTSCTS;
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);

	/* canonical input & output */
	uart_config.c_lflag |= ICANON;
	uart_config.c_lflag &= ~(ECHO | ECHOE | ISIG);
	uart_config.c_oflag |= OPOST;

	/* set baud rate and check for errors*/
	if ((termios_state = cfsetispeed(&uart_config, _baud)) < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}
	if ((termios_state = cfsetospeed(&uart_config, _baud)) < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}
	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void Spingarage::Run() {
	/* make sure the serial port is open */
	open_serial_port();

	/* perform collection */
	int ret_col = collect();
	if (ret_col < 0) {
		PX4_INFO("RETCOL:%d\n", ret_col);
	}
}

void Spingarage::start() {
	/* schedule the driver at regular intervals */
	PX4_INFO("Starting spingarage...\n");
	ScheduleOnInterval(_interval);
}

void Spingarage::stop() {
	/* Ensure the serial port is closed. */
	::close(_fd);

	/* Clear the work queue schedule. */
	ScheduleClear();
}

void Spingarage::print_info() {
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
