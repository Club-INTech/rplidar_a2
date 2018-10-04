#include "SerialCommunication.hpp"
#include "RPLidar.hpp"
#include "LidarEnums.hpp"
#include <unistd.h>
int main(int argc, char** argv){
	RPLidar lidar(argv[argc-1]);
	lidar.set_pwm(660);

	sleep(5);

	lidar.set_pwm(0);
	return 0;
}