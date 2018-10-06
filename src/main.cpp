#include "SerialCommunication.hpp"
#include "RPLidar.hpp"
#include "LidarEnums.hpp"
#include <unistd.h>
int main(int argc, char** argv){
	RPLidar lidar(argv[argc-1]);
	lidar.get_health();
	lidar.get_info();
	lidar.get_samplerate();
	lidar.stop_motor();
	printf("MOTOR START\n");
	lidar.start_motor();
	lidar.start_express_scan();

	sleep(3);

	printf("MOTOR STOP\n");
	lidar.stop_motor();
	return 0;
}