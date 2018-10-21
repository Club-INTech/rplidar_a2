#include <unistd.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <csignal>
#include "LidarWrapper/RPLidar.hpp"
#include "Com/DataSocket.hpp"
static const char* SERVER_ADDRESS=	"127.0.0.1";
static const uint16_t SERVER_PORT=		17685;
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

	DataSocket HL(SERVER_ADDRESS, SERVER_PORT); //Connection to the client
	RPLidar lidar(argc>1?argv[argc - 1]:"/dev/ttyUSB0"); //Connects to lidar
	data_wrappers::FullScan current_scan;
	lidar.print_status(); // Print model, health, sampling rates


	/* ************************************
 	*                   TEST MAIN LOOP             *
 	**************************************/
	while(running) {
		/* ************************************
		 *                   START SCAN                    *
		 **************************************/
		std::cout<<"Waiting for client..."<<std::endl;
		while(!HL.accept_client() && running);
		std::cout<<" Connected !"<<std::endl;
		lidar.stop_scan();
		lidar.start_motor();
		sleep(1);	//Let motor spin
		lidar.start_express_scan();
		int result;
		do{
			lidar.process_express_scans(current_scan);



			result=HL.send_scan(current_scan);
			std::cout<<result<<std::endl;
		}while(result>=0);
		lidar.stop_scan();
		lidar.stop_motor(); //Stop motor if already running
	}
	/* ***********************************
	 *                       STOP ALL                    *
	 *************************************/
	lidar.stop_scan();
	lidar.stop_motor();
	return 0;
}