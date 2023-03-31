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
 * @author Jeremy W. Hopwood
 *
 * Driver for the TriSonica Mini 3D sonic anemometer
 *
 */

#pragma once
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_anemometer.h>

#include <inttypes.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "parser.hpp"


using namespace time_literals;

#define BUFFER_LENGTH 512
#define READ_INTERVAL 100_ms
#define BAUD B115200

class TrisonicaMini : public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 */
	TrisonicaMini(const char *port);
	~TrisonicaMini() override;

	int init();
	void print_info();

private:

	void start();
	void stop();
	void Run() override;
	int measure();
	int collect();

	int _interval{READ_INTERVAL}; // TODO: determine what this should be.
	bool _collect_phase{false};

	int open_serial_port(const speed_t speed = BAUD);

	char _port[20] {};
	int _file_descriptor{-1};
	char _buffer[BUFFER_LENGTH] {};
	int _buffer_index{0};
	hrt_abstime _last_read{0};
	int _parse_state{0};

	unsigned _consecutive_fail_count;

	uORB::PublicationMulti<sensor_anemometer_s> _sensor_anemometer_pub{ORB_ID(sensor_anemometer)};

	perf_counter_t	_sample_perf;
	perf_counter_t	_comms_errors;

};
