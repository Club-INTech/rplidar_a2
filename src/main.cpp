#include "RPLidar.hpp"
#include <unistd.h>
#include <csignal>

using namespace data_wrappers;
bool running=true;
void signal_handler(int signo){
	if(signo==SIGTERM || signo==SIGINT)
		running = false;
}

int main(int argc, char** argv){
	/* ************************************
	*    			SETUP PROGRAM  				 *
	**************************************/

	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);

	/* ************************************
	*    SETUP LIDAR & CHECK STATUS  *
	**************************************/
	RPLidar lidar(argv[argc-1]); //Connects to lidar,

	lidar.print_status(); // Print model, health, sampling rates

	lidar.stop_motor(); //Stop motor if already running


	/* ************************************
	 *                   START SCAN                    *
	 **************************************/
	lidar.start_motor();
	lidar.start_express_scan();
	std::vector<uint8_t> scan_data;
	while(running){
		lidar.read_scan_data(scan_data);
		for(uint8_t data_byte : scan_data){
			printf("%#02x ",data_byte);
		}
		printf("\n");

		scan_data.clear();
	}

	/*
	 * STOP ALL
	 */
	printf("Stopping LiDAR\n");
	lidar.stop_scan();
	lidar.stop_motor();
	return 0;
}