/**
 * @file 	airdata.h
 * @author 	Jeremy Hopwood <jeremyhopwood@vt.edu>
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/airdata.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>

#include <sys/stat.h>
#include <sys/ioctl.h>

#include <drivers/drv_hrt.h>

#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

// Verbose and Packet modes for debugging
// #define VERBOSE
// #define PRINT_PACKET

// Start and end of packet characters
#define START_PACKET '$'
#define END_PACKET '\n'

// Serial port configuration
#define BAUD B115200

// App timeout in seconds (without receiving any data)
#define TIMEOUT 30

/**
 * Global variables
 */
char buffer[1024];
int read_idx = 0;
int write_idx = 0;

using namespace time_literals;

/**
 * Make airdata_main callable as a C function
 */
extern "C" __EXPORT int airdata_main(int argc, char *argv[]);

/**
 * Define the Airdata class derived from the base class, ModuleBase, using the ModuleBase class template
 */
class Airdata : public ModuleBase<Airdata>, public ModuleParams {

	public:

		/** Class constructor */
		Airdata(int example_param, bool example_flag);

		/** Class destructor */
		virtual ~Airdata() = default;

		/** @see ModuleBase */
		static int task_spawn(int argc, char *argv[]);

		/** @see ModuleBase */
		static Airdata *instantiate(int argc, char *argv[]);

		/** @see ModuleBase */
		static int custom_command(int argc, char *argv[]);

		/** @see ModuleBase */
		static int print_usage(const char *reason = nullptr);

		/** @see ModuleBase::run() */
		void run() override;

		/** @see ModuleBase::print_status() */
		int print_status() override;

	private:

		/**
		 * Check for parameter changes and update them if needed.
		 * @param parameter_update_sub uorb subscription to parameter_update
		 * @param force for a parameter update
		 */
		void parameters_update(bool force = false);

		/**
		 * Declare custom parameters, which are defined in @see module.yaml
		 */
		DEFINE_PARAMETERS(
			(ParamInt<px4::params::ADU_PORT>) _adu_port /**< int that defines the port (or to stop the app) */
		)

		/** Subscriptions */
		uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

		/**
		 * Opens and configures the serial port
		 * Returns the port file descriptor or -1 for a critical error
		 */
		int open_serial(const char* device) {

			int port_fd;
			int retn;
			const char* path = device;
			int   baud = BAUD;
			struct termios cfg;

			// Open serial port and return the file descriptor:
			// 	O_RDWR 	  	read-write
			// 	O_NOCTTY   	device does not become the process controlling terminal
			//	O_NONBLOCK 	non-blocking read
			// If the port fails to open, throw error.
			port_fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
			if( port_fd < 0 ) {
				PX4_ERR("failed to open serial port: %s err: %d", path, errno);
				return -1;
			}
			#ifdef VERBOSE
				PX4_INFO("port_fd = %d", port_fd);
			#endif

			// Get the serial port configuration and handle error
			retn = tcgetattr(port_fd, &cfg);
			if( retn < 0 ) {
				PX4_ERR("failed to get port attributes: %d", retn);
				return -1;
			}
			#ifdef VERBOSE
				PX4_INFO("retn_attr = %d", retn);
			#endif

			// Input modes:
			//	~IGNBRK  convert break to null byte
			//	~ICRNL   no CR to NL translation
			//	~INLCR   no NL to CR translation
			//	~PARMRK	 don't mark parity errors or breaks no input parity check
			//	~ISTRIP  don't strip high bit off
			//	~IXON    no XON/XOFF software flow control
			cfg.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

			// Output modes:
			//	no local output processing
			cfg.c_oflag = 0;

			// Local modes
			//	~ECHO    echo off
			//	~ECHONL  echo newline off
			//	~ICANON  canonical mode off
			//	~IEXTEN  extended input processing off
			//	~ISIG    signal chars off
			cfg.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

			// Control modes:
			//	~CSTOPB   1 stop bit (not 2)
			//	~PARENB   no parity
			//	~CRTSCTS  disable flow control
			cfg.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

			// Set input and output baud rates
			retn = cfsetispeed(&cfg, baud);
			if( retn < 0 ) {
				PX4_ERR("failed to set input baud: %d", retn);
				return -1;
			}
			#ifdef VERBOSE
				PX4_INFO("retn_setispeed = %d", retn);
			#endif
			retn = cfsetospeed(&cfg, baud);
			if( retn < 0 ) {
				PX4_ERR("failed to set output baud: %d", retn);
				return -1;
			}
			#ifdef VERBOSE
				PX4_INFO("retn_setospeed = %d", retn);
			#endif

			// Apply new configuration to port
			retn = tcsetattr(port_fd, TCSANOW, &cfg);
			if( retn < 0 ) {
				PX4_ERR("failed to set new configuration: %d", retn);
				return -1;
			}
			#ifdef VERBOSE
				PX4_INFO("retn_setattr = %d", retn);
			#endif

			// return the port file descriptor
			return port_fd;

		}

		/**
		 * Parses serial data and assembles a valid packet containing ADU data
		 * Returns the size of the packet, 0 to wait and try again,
		 * or -1 to exit in the case of a buffer overrun or tiemout.
		 */
		int read_packet(int port_fd, char *packet) {
			// global variables we are utilizing
			// buffer is character array to hold reads (max 1024)
			// write_idx is the write index
			// read_idx in the read index

			int j = 0;
			int overrun_count = 0;
			int ioctl_err_count = 0;

			// initial time (used for timeout)
			uint64_t start_us = hrt_absolute_time();

			while (true) {

				// first use any unused data left in the buffer from a previous read
				int i = read_idx;

				// look for start of packet, throwing away anything else!
				while (j == 0 && i < write_idx) {
					if (buffer[i] == START_PACKET) {
						packet[j++] = buffer[i];
					}
					i++;
				}

				// if there is any data left, copy it into packet until end of packet
				// if we hit another START, start over and throw away what we had
				// if we hit a null, assume it in the end of the buffer
				for ( ; i < write_idx; i++ ) {
					packet[j++] = buffer[i];
					if (buffer[i] == END_PACKET) {
						// we read a whole packet from the existing buffer!
						packet[j] = '\0'; // Null terminate packet
						read_idx  = i+1; // save current read index
						return j; // return size of packet
					}
					if (buffer[i] == START_PACKET) {
						// we received an unexpected START character!
						j           = 0; // start over
						packet[j++] = buffer[i];
						// maybe put debugging data here (multiple start char)
					}
					if (buffer[i] == '\0') {
						// we hit a null after the start packet and before an end packet
						// reset everything
						read_idx  = 0;
						write_idx = 0;
						j         = 0;
					}
					if (j > 200) {
						PX4_ERR("Buffer overrun!");
						packet[j] = '\0';
						if (++overrun_count > 10) {
							return -1; // We've tried enough. Something's wrong.
						}
						return 0; // Try again
					}
				}

				// no more to read reset indexes to beginning of buffer
				read_idx = 0;
				write_idx = 0;

				// get more bytes from the port
				int avail = 0;
				int ioctl_retn;
				ioctl_retn = ioctl(port_fd, FIONREAD, (unsigned long)&avail);
				#ifdef VERBOSE
					PX4_INFO("ioctl_retn = %d", ioctl_retn);
					PX4_INFO("Bytes available: %d", avail);
				#endif

				// handle ioctl errors
				if (ioctl_retn < 0) {
					PX4_ERR("failed to get bytes available: %d", errno);
					if (++ioctl_err_count > 10) {
						return -1; // We've tried enough. Something's wrong.
					}
					return 0; // Try again
				}

				// if no bytes are available, sleep 1000 us and continue.
				if (0 == avail) {
					PX4_WARN("0 bytes available");

					// if no bytes available for TIMEOUT seconds, exit.
					uint64_t now_us = hrt_absolute_time();
					PX4_INFO("sec = %lld", (now_us-start_us)/1000000);
					if ((now_us-start_us)/1000000 >= TIMEOUT) {
						PX4_ERR("No data seen for the specified timeout. Stopping airdata module.");
						return -1;
					}

					usleep(1000); 	// wait a bit for some more bytes to come in
					continue; 	// and try again
				}

				// read the available bytes
				read(port_fd, buffer, avail);

				// move the write buffer
				write_idx = avail;
			}
		}

		/**
		 * Uses the parsed data and publishes it.
		 * Returns 1 if successful, 0 to try again, and -1 to exit the module.
		 */
		int read_airdaq(int port_fd, orb_advert_t adu_pub, struct airdata_s adu) {

			char packet[1024]; // complete packet of data
			int retval; // return value from the parsing of a packet

			// get a high-resolution timestamp in microseconds
			uint64_t timestamp_us = hrt_absolute_time();

			#ifdef VERBOSE
				PX4_INFO("About to read a packet...");
			#endif
			// parse the serial data and get a packet
			retval = read_packet( port_fd, packet );
			#ifdef VERBOSE
				PX4_INFO("Read packet retval = %d", retval);
			#endif

			// If read_packet returns -1, we should exit.
			if (retval < 0) {
				return -1;
			}

			// If it returns 0, warning and try again.
			if (retval == 0) {
				PX4_WARN("Invalid packet: %s. Trying again.", packet);
				return 0;
			}

			float airspeed_kts, pressure_altitude_ft, pitot_pressure_mbar, static_pressure_mbar, alpha_mV, beta_mV;
			double alpha_deg, betaf_deg; // Note: betaf_deg is the measured flank angle

			// get data from packet
			int scan_result = sscanf(
				packet,
				"$ADC,%f,%f,%f,%f,%lf,%lf,%f,%f\r\n",
				&airspeed_kts,
				&pressure_altitude_ft,
				&pitot_pressure_mbar,
				&static_pressure_mbar,
				&alpha_deg,
				&betaf_deg,
				&alpha_mV,
				&beta_mV
			);

			// Check the scan result. If something is wrong at this point, try again.
			if (scan_result != 8) {
				PX4_ERR("Invalid string format from ADU: %d tokens read", scan_result);
				return 0;
			}

			// print the good packet
			#ifdef PRINT_PACKET
				PX4_INFO("%s", packet); // print the packet
			#endif

			// assign values to adu struct
			// const double convFact = M_PI/180.0;
			adu.timestamp = timestamp_us;
			adu.airspeed_kts = airspeed_kts;
			adu.alpha_vane_deg = (float) -alpha_deg;// to account for boom orientation
			adu.beta_vane_deg = (float) betaf_deg; // +beta = wind in the right ear
			adu.pressure_altitude_ft = pressure_altitude_ft;
			adu.pitot_pressure_mbar = pitot_pressure_mbar;
			adu.static_pressure_mbar = static_pressure_mbar;

			// publish the airdata message
			orb_publish(ORB_ID(airdata), adu_pub, &adu);

			// Successfully read a packet of data and published it! Keep reading!
			return 1;

		}

};
