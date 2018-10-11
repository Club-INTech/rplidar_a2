#include "RPLidar.hpp"
#include <unistd.h>
#include <csignal>
#include "q_math.hpp"
#include <cmath>
#include <sys/time.h>

using namespace data_wrappers;

double msecs()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	return (double) tv.tv_usec / 1000 + tv.tv_sec * 1000;
}

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
	bool wrong_flag=false;
	std::vector<uint8_t> read_buffer;
	ExpressPacket packet_current;
	ExpressPacket packet_next;

	// Need two packets for angle computing: current and last (cf formula in p23 of com. protocol documentation)
	double start_time = msecs();
	while(running) {
		read_buffer.clear();
		lidar.read_scan_data(read_buffer, wrong_flag);
		wrong_flag=false;
//		for(uint8_t data_byte : read_buffer){
//			printf("0x%02X ",data_byte);
//		}
//		printf("\n");
		rp_values::ComResult result=packet_current.decode_packet_bytes(read_buffer);
		if(result == rp_values::ComResult::STATUS_WRONG_FLAG){
			printf("WRONG FLAGS, WILL SYNC BACK\n");
			wrong_flag=true;
			continue; //Couldn't decode an Express packet (wrong flags or checksum)
		}
		else if(result != rp_values::ComResult::STATUS_OK){
			printf("WRONG CHECKSUM OR COM ERROR, IGNORE PACKET\n");
			continue;
		}

		double duration= msecs()-start_time;
		start_time= msecs();
		printf("New Packet: Start_Angle=%f\n", packet_current.start_angle);
		printf("Delta_t = %dms\n", static_cast<uint32_t>(duration));
	}

	/*
	 * STOP ALL
	 */
	printf("Stopping LiDAR\n");
	lidar.stop_scan();
	lidar.stop_motor();
	return 0;
}