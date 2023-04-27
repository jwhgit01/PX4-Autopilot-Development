/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/airdata.h>

class PX4AirDataSystem
{
public:
	PX4AirDataSystem(const uint32_t device_id, const float_t device_position[3] = {0});
	~PX4AirDataSystem();

	// Set the MAV_AIR_DATA_SYSTEM type (VANED, SONIC, PROBE)
	void set_air_data_system_type(uint8_t air_data_system_type) { _airdata_pub.get().type = air_data_system_type; };

	void set_device_id(const uint32_t device_id) { _airdata_pub.get().device_id = device_id; };
	void set_device_type(const uint8_t device_type);

	void set_position(const float_t device_position[3] = {0});

	void update(const hrt_abstime &timestamp_sample, float V, float beta, float alpha);

	int get_instance() { return _airdata_pub.get_instance(); };

private:
	uORB::PublicationMultiData<airdata_s> _airdata_pub{ORB_ID(airdata)};
};
