#include "RPLidar.hpp"
#include <unistd.h>
#include <csignal>
#include <chrono>
#include "q_math.hpp"
#include <cmath>
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

	/* ************************************
 	*                   TEST MAIN LOOP             *
 	**************************************/
	bool wrong=false;
	std::vector<uint8_t> read_buffer;
	ExpressPacket packet_current;
	ExpressPacket packet_next;

	// Need two packets for angle computing: current and last (cf formula in p23 of com. protocol documentation)
	auto start=std::chrono::steady_clock::now();
	while(running) {
		read_buffer.clear();
		lidar.read_scan_data(read_buffer);
//		for(uint8_t data_byte : read_buffer){
//			printf("0x%02X ",data_byte);
//		}
//		printf("\n");
		if(!packet_current.decode_packet_bytes(read_buffer)){
			continue;
		}

		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
		printf("New Packet: Start_Angle=%f\n", packet_current.start_angle);
		printf("delta_t=%lims\n", duration.count());
		start=std::chrono::steady_clock::now();
	}

	/*
	 * STOP ALL
	 */
	printf("Stopping LiDAR\n");
	lidar.stop_scan();
	lidar.stop_motor();
	return 0;
}