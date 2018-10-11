#include "RPLidar.hpp"
#include <unistd.h>
#include <cmath>

using namespace data_wrappers;



int main(int argc, char** argv){
	/* ************************************
	*    SETUP LIDAR & CHECK STATUS  *
	**************************************/
	RPLidar lidar(argc>1?argv[argc - 1]:"/dev/ttyUSB0"); //Connects to lidar,

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
	lidar.process_express_scans();

	/* ***********************************
	 *                       STOP ALL                    *
	 *************************************/
	lidar.stop_scan();
	lidar.stop_motor();
	return 0;
}