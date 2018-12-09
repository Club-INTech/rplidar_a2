#include <unistd.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <csignal>
#include "LidarWrapper/RPLidar.hpp"

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

	RPLidar lidar(argc>1?argv[argc - 1]:"/dev/ttyUSB0"); //Connects to lidar
	lidar.print_status(); // Print model, health, sampling rates
	lidar.start();
	/* ************************************
 	*                   TEST MAIN LOOP             *
 	**************************************/
	while(running) {
		/* ************************************
		 *                   START SCAN                    *
		 **************************************/
		lidar.update();
		lidar.print_scan();
	}
	/* ***********************************
	 *                       STOP ALL                    *
	 *************************************/
	lidar.stop();
	return 0;
}