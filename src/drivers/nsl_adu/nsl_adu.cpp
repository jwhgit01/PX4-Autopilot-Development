/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

#include "nsl_adu.hpp"

NSL_ADU::NSL_ADU(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config) {
}

NSL_ADU::~NSL_ADU() {
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int NSL_ADU::init() {
	// I2C init
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	start();
	return PX4_OK;
}

void NSL_ADU::start() {
	// Schedule a cycle to start things.
	_collect_phase = false;
	ScheduleDelayed(_interval);
}

int NSL_ADU::collect() {

	perf_begin(_sample_perf);

	// Get timestamp
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// trasfer NUM_BYTES bytes into _val and check for errors
	uint8_t _val[_num_bytes] {};
	if (transfer(nullptr, 0, &_val[0], _num_bytes) < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	// Copy the bytes into the float variables using memcpy
	float beta_rad;
	float alpha_rad;
	if (sizeof(float) * 2 <= _num_bytes) {
		union FloatStorage {
        		uint8_t bytes[sizeof(float)];
        		float value;
    		};

		FloatStorage beta_storage;
    		std::memcpy(beta_storage.bytes, _val, sizeof(float));
    		beta_rad = beta_storage.value;

    		FloatStorage alpha_storage;
    		std::memcpy(alpha_storage.bytes, _val + sizeof(float), sizeof(float));
    		alpha_rad = alpha_storage.value;
	} else {
    		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	// Publish the data!
	sensor_flow_angle_s sensor_flow_angle{};
	sensor_flow_angle.timestamp = timestamp_sample;
	sensor_flow_angle.flank_angle_rad = beta_rad;
	sensor_flow_angle.angle_of_attack_rad = alpha_rad;
	_sensor_flow_angle_pub.publish(sensor_flow_angle);

	perf_end(_sample_perf);

	return PX4_OK;
}

int NSL_ADU::measure() {
	// send the command to begin a conversion.
	uint8_t cmd = ADDR_READ;
	int ret = transfer(&cmd, 1, nullptr, 0);
	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}
	return PX4_OK;
}

void NSL_ADU::RunImpl() {
	// Collect data if needed
	if (_collect_phase) {
		// Perform collection.
		if (OK != collect()) {
			perf_count(_comms_errors);
			_collect_phase = false;
			ScheduleNow();
			return;
		}

		// Next phase is measurement
		_collect_phase = false;

		// Schedule a fresh cycle call when we are ready to measure again
		ScheduleDelayed(_interval);
		return;
	}

	// Perform measurement
	if (OK != measure()) {
		PX4_DEBUG("NSL_ADU measure error");
	}

	// Next phase is collection
	_collect_phase = true;

	// Schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(_interval);

}

void NSL_ADU::print_status() {
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
