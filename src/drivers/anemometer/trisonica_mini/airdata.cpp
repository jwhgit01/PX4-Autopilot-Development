/**
 * @file 	airdata.cpp
 *			Spingarage air data unit PX4 app that parses serial data and publishes it as a uORB message
 * @author 	Jeremy Hopwood <jeremyhopwood@vt.edu>
 */

#include "airdata.h"

/**
 * Print runtime information about the state of the module.
 */
int Airdata::print_status() {
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

/**
 * Define any custom commands that are used in the instantiation of an Airdata object.
 * This is a required method of the base class, ModuleBase.
 */
int Airdata::custom_command(int argc, char *argv[]) {
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}

/**
 * Run the app as a task on its own stack with default priority.
 */
int Airdata::task_spawn(int argc, char *argv[]) {
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      3072,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if( _task_id < 0 ) {
		_task_id = -1;
		return -errno;
	}

	#ifdef VERBOSE
		PX4_INFO("Task spawned successfully.");
	#endif

	return 0;
}

/**
 * Methods for instantiating an airdata object with added functionality
 * This is where we parse flags and parameters (i.e. $ airdata start -f -p 0)
 */
Airdata *Airdata::instantiate(int argc, char *argv[]) {
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if( error_flag ) {
		return nullptr;
	}

	Airdata *instance = new Airdata(example_param, example_flag);

	if( instance == nullptr ) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

/**
 * This is the constructor of the Airdata class using the parameters and flags above
 */
Airdata::Airdata(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

/**
 * This is where the main function is written. It gets called in the static function,
 * run_trampoline, within the base class, ModuleBase.
 */
void Airdata::run() {

	// wait a second
	sleep(1);

	// check to see if we should exit
	if (should_exit()) {
		PX4_ERR("Module told to exit from outside of main function. Exiting.");
		return; // stop the module
	}

	// initialize parameters
	parameters_update(true);
	#ifdef VERBOSE
		PX4_INFO("Parameters initialized");
	#endif

	// get the port parameter
	// TODO: get the port numbering from the board cmake file (currently compatible with just cubeorange)
	int32_t adu_port = 0;
	adu_port = _adu_port.get();
	const char* adu_path;
	switch (adu_port) {
		case 0: {
			// OFF
			adu_path = "0";
			// todo: function to stop module
			return;
		} break;
		case 1: {
			// TELEM1 (ttyS0)
			adu_path = "/dev/ttyS0";
		} break;
		case 2: {
			// TELEM2 (ttyS1)
			adu_path = "/dev/ttyS1";
		} break;
		case 3: {
			// GPS1 (ttyS2)
			adu_path = "/dev/ttyS2";
		} break;
		case 4: {
			// GPS2 (ttyS5)
			adu_path = "/dev/ttyS5";
		} break;
		default: {
			PX4_ERR("Invalid port!");
			adu_path = "0";
			// todo: function to stop module
			return;
		}
	}

	// open the serial port
	int port_fd;
	PX4_INFO("Attempting to open ADU on %s", adu_path);
	port_fd = Airdata::open_serial(adu_path);
	if (port_fd < 0) {
		return; // stop the module
	}
	PX4_INFO("Port opened with file descriptor %d", port_fd);
	usleep(10000);

	// Declare and safely initialize uORB message struct to zero
	struct airdata_s adu;
	memset(&adu, 0, sizeof(adu));

	// advertise airdata uORB topic
	orb_advert_t adu_pub = orb_advertise(ORB_ID(airdata), &adu);
	#ifdef VERBOSE
		PX4_INFO("The airdata topic has been advertised.");
	#endif

	// while everything is OK, keep reading data
	int adu_status = 0;
	while (!should_exit()) {

		// read and publish adu data
		adu_status = Airdata::read_airdaq(port_fd, adu_pub, adu);
		#ifdef VERBOSE
			PX4_INFO("We read and published some data!");
		#endif

		// If read_airdaq returns -1, something is wrong. Exit.
		if (adu_status < 0) {
			PX4_ERR("Something wrong with Air Data Unit! Exiting.");
			return; // stop the module
		}

		// update parameters
		parameters_update();
		#ifdef VERBOSE
			PX4_INFO("Parameters updated");
		#endif

		// Check for change of ADU_PORT to 0 (OFF)
		if (_adu_port.get() < 1) {
			PX4_INFO("ADU_PORT = 0: Stop module.");
			return; // stop the module
		}
		// TODO: Enable hot-switching the port

		// sleep 200 us for stability (try without this)
		usleep(200);
	}

	// close the serial port
	close(port_fd);
}

/**
 * Check for and update parameters
 */
void Airdata::parameters_update(bool force) {
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

/**
 * Define the usage/implementation of this module as well as documentation for it
 */
int Airdata::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ airdata start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airdata", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

/**
 * This is the function the can be called in C. Airdata::main is defined in the
 * base class, ModuleBase, which handles start, stop, status, and custom commands.
 */
int airdata_main(int argc, char *argv[]) {
	return Airdata::main(argc, argv);
}
