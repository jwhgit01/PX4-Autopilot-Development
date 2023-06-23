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

/**
 * @file nsl_adu.cpp
 *
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 *
 * Driver for Virginia Tech Nonlinear Systems Laboratory vaned air data unit.
 * Default I2C address is 0x4F.
 */

#include "nsl_adu.hpp"

//NSL_ADU::NSL_ADU(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
//	I2C(DRV_AIRDATA_DEVTYPE_NSL_ADU, MODULE_NAME, bus, address, bus_frequency),
//	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
//	_px4_air_data_system(0, float [3] {0}) {
//}
NSL_ADU::NSL_ADU(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config) {
}

NSL_ADU::~NSL_ADU() {
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void NSL_ADU::start() {
	// Schedule a cycle to start things.
	_collect_phase = false;
	ScheduleDelayed(_interval);
}

int NSL_ADU::init() {
	// I2C init (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}
	return measure();
}

/**
 * @brief 	Process the requested bytes
 * @details	Process the incoming bytes after we have requested a read.
 * 			The bytes are mapped to floats of V, alpha, beta, and then
 * 			are used to update a PX4airdata object, PX4AirDataSystem object,
 * 			defined in ~/src/lib/drivers/PX4AirDataSystem/
 */
int NSL_ADU::collect() {

	perf_begin(_sample_perf);

	/**
	 * @section Read from the sensor
	 */
	//
	// Get timestamp
	//
	uint8_t _val[_num_bytes] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	//
	// trasfer NUM_BYTES bytes into _val and check for errors
	//
	if (transfer(nullptr, 0, &_val[0], _num_bytes) < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	/**
	 * @section Re-assemble the bytes into floats and update UORB topic
	 */
	float beta_rad;
	float alpha_rad;

	// Copy the bytes into the float variables using memcpy and reinterpret_cast
	if (sizeof(float) * 2 <= _num_bytes) {
    		std::memcpy(&beta_rad, reinterpret_cast<const float*>(_val), sizeof(float));
    		std::memcpy(&alpha_rad, reinterpret_cast<const float*>(_val + sizeof(float)), sizeof(float));
	} else {
    		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	// Update the PX4airdata object to be published (TODO: sub to airspeed)
	_px4_air_data_system.update(timestamp_sample, 0.0f, beta_rad, alpha_rad);

	perf_end(_sample_perf);

	return PX4_OK;
}

/**
 * @brief Initiate a measurement by sending 0x00
 */
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
			PX4_DEBUG("collection error");
			// If error restart the measurement state machine.
			start();
			return;
		}

		// Next phase is measurement.
		_collect_phase = false;
	}

	// Perform measurement.
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	// Next phase is collection.
	_collect_phase = true;

	// Schedule a fresh cycle call when the measurement is done.
	ScheduleDelayed(_interval);
}

/**
 * @brief Print the status of the driver using the base I2C class
 */
void NSL_ADU::print_status() {
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
