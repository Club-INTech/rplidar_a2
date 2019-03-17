#include <csignal>
#include "lidar/RPLidar.hpp"

bool running=true; //Variable to manage stops when signals are received
void signal_handler(int signo){
	if(signo==SIGTERM || signo==SIGINT)
		running = false;
}

int main(int argc, char** argv){
	/* ************************************
	 *      SETUP LIDAR & CHECK STATUS    *
	 **************************************/
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);
	RPLidar lidar; //Connects to lidar
	running = lidar.init(argc>1?argv[argc - 1]:"/dev/ttyUSB0");
	/* ************************************
	 *               START 			      *
	 **************************************/
	if(running) {
		lidar.print_status();
		running = lidar.start();
	}

	while(running){
		//Update current scan (one turn of measurements)
		lidar.update();
		lidar.print_scan();
		lidar.print_deltas();
	}
	/* ***********************************
	 *              STOP ALL             *
	 *************************************/
	return 0;
}