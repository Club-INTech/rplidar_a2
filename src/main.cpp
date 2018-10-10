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
	std::vector<uint8_t> scan_data;
	auto start=std::chrono::steady_clock::now();
	while(running) {
		scan_data.clear();
		lidar.read_scan_data(scan_data);
		if (((scan_data[0] & 0xF0) != 0xA0) || ((scan_data[1] & 0xF0) != 0x50)) {
			continue;
		}
		uint8_t read_checksum = (scan_data[0] & 0x0F) | ((scan_data[1] & 0x0F) << 4);
		uint8_t checksum = scan_data_checksum(scan_data);
		if (read_checksum != checksum) {
//			printf("ERROR : WRONG CHECKSUM GOT 0x%02X EXPECTED 0x%02X\n", checksum, read_checksum);
			continue;
		}
//		for(uint8_t data_byte : scan_data){
//			printf("0x%02X ",data_byte);
//		}
//		printf("\n");
		float angle = (((scan_data[3] & 0x7F) << 4) | (scan_data[2])) / 64.0;
//		float angle=q_div(angle_q, 64, 6);

		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
		if ((scan_data[3] & 0x80) > 0)
			printf("NEW SCAN\n");
		else {
			printf("byte 2 3 4: 0x%02X\n", scan_data[3]);
			continue;
		}
		for(uint8_t data_byte : scan_data){
			printf("0x%02X ",data_byte);
		}
		printf("\n");
//		printf("delta_t=%lims new_scan=%#02x start angle=%f\n", duration.count(), scan_data[3], angle);
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