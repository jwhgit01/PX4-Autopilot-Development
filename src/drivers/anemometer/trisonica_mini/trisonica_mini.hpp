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
 * @file lightware_laser_serial.hpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 *
 * Driver for the Lightware laser rangefinder series
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

//#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_anemometer.h>
#include <uORB/topics/sensor_pth.h>

#include <inttypes.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

using namespace time_literals;

class TrisonicaMini : public px4::ScheduledWorkItem
{
public:
	TrisonicaMini(const char *port);
	~TrisonicaMini() override;

	int init();
	void print_info();
	int parse(const char* packet_data);

private:

	/* Standard PX4 Driver functions */
	void start();
	void stop();
	void Run() override;
	int collect();
	int open_serial_port();

	/* Serial data parser */
	enum PARSE_STATE {
		PARSE_STATE_UNSYNC = 0,
		PARSE_STATE_SYNC,
		PARSE_STATE_GOT_S
	};
	//int assemble(char c, unsigned *_readbuf_index, char *packetbuf, unsigned *packetbuf_index, enum PARSE_STATE *state);
	//int parse(char c, char *packetbuf, unsigned *packetbuf_index, enum PARSE_STATE *state, float *dist);

	// PX4Rangefinder _px4_rangefinder;

	char _port[20] {};
	unsigned _baud{B115200};
	int _interval{10_ms};
	int	_fd{-1};
	char _packet[128] {};
	unsigned _readbuf_idx = 0;
	unsigned _packet_idx = 0;
	hrt_abstime _last_read{0};
	//In a packet, the first character of
	//the anemometer sensor output is "S"
	const char _starting_char = 'S';
	//In a packet, the last character of
	//the anemometer sensor output is a line feed ('\n')
	const char _ending_char = '\n';
	unsigned _consecutive_fail_count;
	bool _assemble_packet = 0; //boolean to start assembling the packet
	int _overrun_count = 0; //Buffer overrun counter

	//High-resolution data timestamp in microseconds
	uint64_t timestamp_us;

	//Anemometer structs
	//struct sensor_anemometer_s anem_uvw;
	//struct sensor_pth_s anem_pth;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	uORB::PublicationMulti<sensor_anemometer_s> _sensor_anemometer_pub{ORB_ID(sensor_anemometer)};
	uORB::PublicationMulti<sensor_pth_s> _sensor_pth_pub{ORB_ID(sensor_pth)};
	//orb_advert_t sensor_anemometer_pub; //Test
	//orb_advert_t sensor_pth_pub; //Test
};
