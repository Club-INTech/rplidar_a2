#include "LidarWrapper/RPLidar.hpp"
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <csignal>

using namespace data_wrappers;
using namespace std;

bool running=true; //Variable to manage stops when signals are received
void signal_handler(int signo){
	if(signo==SIGTERM || signo==SIGINT)
		running = false;
}

int main(int argc, char** argv){
	/* ************************************
	*    SETUP LIDAR & CHECK STATUS  *
	**************************************/
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);
	running=true;

	RPLidar lidar(argc>1?argv[argc - 1]:"/dev/ttyUSB0"); //Connects to lidar,
	data_wrappers::FullScan current_scan;
	lidar.print_status(); // Print model, health, sampling rates

	lidar.stop_motor(); //Stop motor if already running

	/* ************************************
	 *                   START SCAN                    *
	 **************************************/
	lidar.start_motor();
	sleep(2);	//Let motor spin
	lidar.start_express_scan();

	/* ************************************
 	*                   TEST MAIN LOOP             *
 	**************************************/
	while(running) {
		lidar.process_express_scans(current_scan);

		lidar.print_scan(current_scan);

	}
	/* ***********************************
	 *                       STOP ALL                    *
	 *************************************/
	lidar.stop_scan();
	lidar.stop_motor();
	return 0;
}