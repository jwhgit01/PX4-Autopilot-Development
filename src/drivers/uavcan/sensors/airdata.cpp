/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @author Jeremy Hopwood <jeremyhopwood@vt.edu>
 */

#include "airdata.hpp"

const char *const UavcanAirdataBridge::NAME = "airdata";

UavcanAirdataBridge::UavcanAirdataBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_airdata", ORB_ID(sensor_airdata)),
	_sub_airspeed_data(node),
	_sub_flowangle_data(node)
{ }

int UavcanAirdataBridge::init()
{
	int res = _sub_airspeed_data.start(AirspeedCbBinder(this, &UavcanAirdataBridge::airspeed_sub_cb));
	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	int res2 = _sub_beta_data.start(FlowAngleCbBinder(this, &UavcanAirdataBridge::flowangle_sub_cb));
	if (res2 < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res2);
		return res2;
	}

	return 0;
}

void UavcanAirdataBridge::airspeed_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::TrueAirspeed> &msg)
{
	_last_airspeed_m_s = msg.true_airspeed;
}

void UavcanAirdataBridge::flowangle_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::FlowAngleData> &msg)
{
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (channel == nullptr) {
		// Something went wrong - no channel to publish on; return
		return;
	}

	// Cast our generic CDev pointer to the sensor-specific driver class
	PX4AirDataSystem *air_data_system = (PX4AirDataSystem *)channel->h_driver;

	if (air_data_system == nullptr) {
		return;
	}

	air_data_system->update(timestamp_sample, _last_airspeed_m_s, msg.beta_vane, msg.alpha_vane);
}

int UavcanAirdataBridge::init_driver(uavcan_bridge::Channel *channel)
{
	// update device id as we now know our device node_id
	DeviceId device_id{_device_id};

	device_id.devid_s.devtype = DRV_ACC_DEVTYPE_UAVCAN;
	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

	channel->h_driver = new PX4AirDataSystem(device_id.devid);

	if (channel->h_driver == nullptr) {
		return PX4_ERROR;
	}

	PX4AirDataSystem *air_data_system = (PX4AirDataSystem *)channel->h_driver;

	channel->instance = air_data_system->get_instance();

	if (channel->instance < 0) {
		PX4_ERR("UavcanAirdata: Unable to get a class instance");
		delete air_data_system;
		channel->h_driver = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}
