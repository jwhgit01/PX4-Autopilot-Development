/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file trisonica_mini.hpp
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 * @author Nazmus Sakib <>
 *
 * Driver for the Trisonica Mini 3D sonic anemometer
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_anemometer.h>
#include <uORB/topics/sensor_pth.h>

#include <inttypes.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

using namespace time_literals;

/* constants */
static constexpr float MBAR2PA = 100.0; // millibar to Pascals
static constexpr unsigned PACKETLEN = 99; // The length of a packet including null termination

class TrisonicaMini : public px4::ScheduledWorkItem
{
public:
	TrisonicaMini(const char *port);
	~TrisonicaMini() override;

	int init();
	void print_info();

private:

	/* Standard PX4 Driver functions */
	void start();
	void stop();
	void Run() override;
	int collect();
	int open_serial_port();

	/* Function for parsing packet data */
	int parse(const char* packet_data);

	/* Private variables */
	char _port[20] {}; // Serial port name
	unsigned _baud{B115200}; // Baud rate of the Trisonical Mini
	int _interval{10_ms}; // Read interval. The Trisonica Mini sampling rate is 10 Hz.
	int	_fd{-1}; // Serial port file descriptor
	unsigned _readlen = PACKETLEN - 1; // The number of bytes to read
	char _packet[PACKETLEN] {}; // A buffer for assembling a packet of data
	unsigned _packet_idx = 0; // The packet index (where the valid bytes read are placed)
	hrt_abstime _last_read{0}; // Time of the last read used for error checking
	const char _starting_char = 'S'; // Starting character of a packet
	const char _ending_char = '\n'; // Ending character of a packet
	bool _assemble_packet = 0; // Boolean whether to start assembling the packet
	int _overrun_count = 0; // Buffer overrun counter
	int _device_ID; // PX4 device ID

	uint64_t timestamp_us; // High-resolution data timestamp in microseconds

	perf_counter_t _sample_perf; // Number of samples performed
	perf_counter_t _comms_errors; // Number of sample errors

	uORB::PublicationMulti<sensor_anemometer_s> _sensor_anemometer_pub{ORB_ID(sensor_anemometer)};
	uORB::PublicationMulti<sensor_pth_s> _sensor_pth_pub{ORB_ID(sensor_pth)};
};
