/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
 * 						2022 Jeremy W. Hopwood
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

namespace trisonica_mini
{

TrisonicaMini *g_dev{nullptr};

static int start(const char *port)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	if (port == nullptr) {
		PX4_ERR("no device specified");
		return -1;
	}


	/* create the driver */
	g_dev = new TrisonicaMini(port);

	if (g_dev == nullptr) {
		return -1;
	}

	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		return -1;
	}

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	g_dev->print_info();

	return 0;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial  driver for the TriSonica Mini 3D sonic anemometer.

Select boards are configured to enable/start the driver on a specified UART using the SENS_TRISONICA_CFG parameter.

### Examples

Attempt to start driver on a specified serial device.
$ trisonica_mini start -d /dev/ttyS1
Stop driver
$ trisonica_mini stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trisonica_mini", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("anemometer");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int trisonica_mini_main(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		default:
			trisonica_mini::usage();
			return -1;
		}
	}

	if (myoptind >= argc) {
		trisonica_mini::usage();
		return -1;
	}

	if (!strcmp(argv[myoptind], "start")) {
		return trisonica_mini::start(device_path);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return trisonica_mini::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return trisonica_mini::status();
	}

	trisonica_mini::usage();
	return -1;
}
