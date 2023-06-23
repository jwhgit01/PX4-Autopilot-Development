/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file nsl_adu.hpp
 *
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/air_data_system/PX4AirDataSystem.hpp>
// OR:
//#include <uORB/Publication.hpp>
//#include <uORB/PublicationMulti.hpp>
//#include <uORB/topics/sensor_airdata.h>
#include <drivers/drv_hrt.h>
#include <cstdint>

/**
 * Configuration Constants
 */
static constexpr uint8_t ADU_ADDRESS = 0x4F;
static constexpr uint8_t ADDR_READ = 0x00;
static constexpr unsigned MEAS_RATE = 200; // Hz
static constexpr int64_t CONVERSION_INTERVAL = (1000000 / MEAS_RATE); /* microseconds */

class NSL_ADU : public device::I2C, public I2CSPIDriver<NSL_ADU>
{
public:
	NSL_ADU(const I2CSPIDriverConfig &config);
	//NSL_ADU(I2CSPIBusOption bus_option,
	//	const int bus,
	//	int bus_frequency,
	//	int address = ADU_ADDRESS);

	~NSL_ADU() override = default;

	//static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli,
	//				     const BusInstanceIterator &iterator,
	//				     int runtime_instance);

	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();

private:

	void start();
	int collect();
	int measure();

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 * @param address The I2C bus address to probe.
	 * @return True if the device is present.
	 */
	int probe_address(uint8_t address);

	PX4AirDataSystem _px4_air_data_system;
	// OR uORB::PublicationMulti<sensor_airdata_s> _rpm_pub{ORB_ID(sensor_airdata)};

	uint32_t _interval{CONVERSION_INTERVAL};

	bool _collect_phase{false};
	static constexpr int _num_bytes = 8;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED,  MODULE_NAME": read")};
};
