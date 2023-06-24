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

#include "trisonica_mini.hpp"

TrisonicaMini::TrisonicaMini(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* set device ID */
	//device::Device::DeviceId device_id;
	//evice_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	/* serial bus number (assuming '/dev/ttySx') */
	// uint8_t bus_num = atoi(&_port[strlen(_port) - 1]);
	//if (bus_num < 10) {
	//	device_id.devid_s.bus = bus_num;
	//}

	// _px4_rangefinder.set_device_id(device_id.devid);
	// _px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LIGHTWARE_LASER);
}

TrisonicaMini::~TrisonicaMini() {
	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TrisonicaMini::init() {
	start();
	return PX4_OK;
}

/*
int TrisonicaMini::assemble(char c, char *parserbuf, unsigned *parserbuf_index, enum PARSE_STATE *state){
	TODO
}

int TrisonicaMini::parse(TODO){
	TODO
}
*/

int TrisonicaMini::parse(const char* packet_data)
{
	//Read and assign values to the data (!!!!!!FIX UNITS!!!!!!!!!)
	float windspeed_mps, wind_direction_deg, U_vel_mps, V_vel_mps, W_vel_mps, temperature_celcius, humidiy_precent, pressure_Pa;
	double air_density_kgpm3;
	int scan_result = sscanf(packet_data,
	"S: %f,D:%f,U:%f,V:%f,W:%f,T:%f,H:%f,P:%f,AD:%lf\n\n",
				&windspeed_mps,
				&wind_direction_deg,
				&U_vel_mps,
				&V_vel_mps,
				&W_vel_mps,
				&temperature_celcius,
				&humidiy_precent,
				&pressure_Pa,
				&air_density_kgpm3);
	//Check the scan result. If something is wrong at this point, try again.
	if (scan_result != 9) {
		PX4_ERR("Invalid string format from Anemometer: %d tokens read", scan_result);
		return 0;
	}

	/*
	// print the good packet
	#ifdef PRINT_PACKET
		PX4_INFO("%s", packet_data); //Print the packet
	#endif
	*/

	//Assign values to the "sensor_anemometer" message

	
	//uint devID = Convert.ToUInt32(_fd);
	anem_uvw.device_id = (uint) 0;
	anem_uvw.timestamp = timestamp_us;
	anem_uvw.u = U_vel_mps;
	anem_uvw.v = V_vel_mps;
	anem_uvw.w = W_vel_mps;
	//Publish
	orb_publish(ORB_ID(sensor_anemometer), sensor_anemometer_pub, &anem_uvw);

	// Successfully read a packet of data and published it! Keep reading!
	return 1;

}

int TrisonicaMini::collect() {

	PX4_INFO("Starting collection...\n");

	/* TODO: what is this and is it needed? */
	//perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_packet)];
	unsigned readlen = sizeof(readbuf) - 1;

	/* read from the sensor (uart buffer) */
	//const hrt_abstime timestamp_sample = hrt_absolute_time();
	int bytes_read = ::read(_fd, &readbuf[_readbuf_idx], readlen);
	if (bytes_read < 0) {
		PX4_DEBUG("read err: %d", bytes_read);
		//perf_count(_comms_errors);
		//perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_interval * 2)) {
			return bytes_read;
		} else {
			return -EAGAIN;
		}
	} else if (bytes_read == 0) {
		/* zero bytes read without error. read again. */
		return -EAGAIN;
	}

	PX4_INFO("bytes read:\n");

	for (int i = _readbuf_idx; i < bytes_read; i++)
	{
		//Look for the start of the packet
		if (_assemble_packet == 0 && readbuf[i] == _starting_char)
		{
			_packet[_packet_idx++] = readbuf[i];
			timestamp_us = hrt_absolute_time(); //Get the data timestamp in microseconds
			_assemble_packet = 1;
		}

		//Assemble data into packet when a start character is found
		if (_assemble_packet == 1)
		{
			// Copy all the subsequent data into the packet
			_packet[_packet_idx++] = readbuf[i];
			//If the ending character show up, finish reading the packet
			if (readbuf[i] == _ending_char)
			{
				_packet[_packet_idx] = '\0'; //Null terminate the packet, that is the standard
				_assemble_packet = 0; //Stop assembling packet

				//****DO SOMETHING WITH THE DATA****//

				//Advance the read index two steps if there are two null characters
				_readbuf_idx++;
				return _packet_idx; //return size of packet
				//PX4_INFO("Found LF: %x at %d",readbuf[i], i);
			}

			//If we receive another start character (corrupted data!)
			else if (readbuf[i] == _starting_char)
			{
				_packet_idx = 0; //Start over
				_packet[_packet_idx++] = readbuf[i];
				timestamp_us = hrt_absolute_time(); //Get the data timestamp in microseconds
				_assemble_packet = 1;
			}

			//If we hit a null ('\0') character after starting (corrupt data!)
			else if	(readbuf[i] == '\0')
			{
				//Reset everything
				_readbuf_idx = 0;
				_packet_idx = 0;
				bytes_read = 0;
				_assemble_packet = 0;
			}

			else if (_packet_idx > 200)
			{
				PX4_ERR("Buffer overrun!");
				_overrun_count++; //Increase the counter
				_packet[_packet_idx] = '\0'; //Terminate with a null
				if (_overrun_count > 10)
				{
					return -1; //Exit out, something is wrong
				}
				return 0; //Try again
			}
		}
	}

	//No more data to read (end of buffer reached)
	_readbuf_idx = 0; //Reset the read index to 0
	bytes_read = 0;
	PX4_INFO("End of data \n");

	/* we have made a successful read. make note of the time and initialize. */
	//_last_read = hrt_absolute_time();
	//bool valid = false;

	/* TODO: Determine how we should parse the data based on the mode */
	// int32_t mode = 0;
	// param_get(param_find("SENS_EN_TRIMINI"), &mode);

	/* assemble a packet of raw bytes
	for (size_t i = 0; i < bytes_read; i++) {
		if (OK == TrisonicaMini::assemble(readbuf[i], &_readbuf_idx, _packet, &_packet_index, &_parse_state)) {
			valid = true;
		}
	}
	if (!valid) {
		return -EAGAIN;
	} */

	//float <empty topic here> = <static const empty value>; // TODO:

	/* supposedly we have a complete packet we can now parse */
	//int ret = TrisonicaMini::parse(_packet, &topic);

	// _px4_rangefinder.update(timestamp_sample, distance_m);

	//perf_end(_sample_perf);

	/*
	 * Here, we will publish to two topics: sensor_anemometer and sensor_pth
	 */

	return PX4_OK;
}

int TrisonicaMini::open_serial_port() {
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
	if (_fd < 0)
	{
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}
	if (!isatty(_fd))
	{
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	struct termios uart_config;
	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_fd, &uart_config);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;		/* 8-bit characters */
	uart_config.c_cflag &= ~PARENB; /* no parity bit */
	uart_config.c_cflag &= ~CSTOPB; /* only need 1 stop bit */
	uart_config.c_iflag |= ICRNL;  /* CR is a line terminator */
	uart_config.c_iflag |= IGNPAR; // Ignore parity errors

	// no flow control
	uart_config.c_cflag &= ~CRTSCTS;
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);

	// canonical input & output
	uart_config.c_lflag |= ICANON;
	uart_config.c_lflag &= ~(ECHO | ECHOE | ISIG);
	uart_config.c_oflag |= OPOST;

	/* set baud rate and check for errors*/
	if ((termios_state = cfsetispeed(&uart_config, _baud)) < 0)
	{
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}
	if ((termios_state = cfsetospeed(&uart_config, _baud)) < 0)
	{
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}
	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0)
	{
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_fd);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void TrisonicaMini::Run() {
	/* make sure the serial port is open */
	open_serial_port();

	/* perform collection */
	int ret_col = collect();
	if (ret_col < 0) {
		PX4_INFO("RETCOL:%d\n", ret_col);
	}
}

void TrisonicaMini::start() {
	PX4_INFO("Starting trisonica_mini...\n");
	/* schedule the driver at regular intervals */
	ScheduleOnInterval(_interval);
}

void TrisonicaMini::stop() {
	/* Ensure the serial port is closed. */
	::close(_fd);

	/* Clear the work queue schedule. */
	ScheduleClear();
}

void TrisonicaMini::print_info() {
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
