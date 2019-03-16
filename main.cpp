#include <csignal>
#include "lidar/RPLidar.hpp"

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
	lidar.print_status();
	/* ************************************
	 *                         START 			              *
	 **************************************/
	lidar.start();
	while(running){
		//Update current scan (one turn of measurements)
		lidar.update();
		lidar.print_scan();
	}
	/* ***********************************
	 *                       STOP ALL                    *
	 *************************************/
	lidar.stop();
	return 0;
}