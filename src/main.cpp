#include "RPLidar.hpp"
#include <unistd.h>

using namespace data_wrappers;

int main(int argc, char** argv){
	RPLidar lidar(argv[argc-1]);
	HealthData healthData;
	lidar.get_health(&healthData);
	printf("Lidar Health: %s\n", healthData.status==rp_values::LidarStatus::LIDAR_OK?"OK":(healthData.status==rp_values::LidarStatus::LIDAR_WARNING?"WARNING":"ERROR"));
	if(healthData.status>0){
		printf("Warning/Error code: %u\n", healthData.error_code);
	}
	lidar.get_info();
	SampleRateData sampleRateData;
	lidar.get_samplerate(&sampleRateData);
	printf("Scan sampling rate:%u\nExpress sampling rate:%u\n", sampleRateData.scan_sample_rate, sampleRateData.express_sample_rate);
	lidar.stop_motor();
	printf("MOTOR START\n");
	lidar.start_motor();
//	lidar.start_express_scan();

	sleep(3);

//	lidar.stop_scan();
	printf("MOTOR STOP\n");
	lidar.stop_motor();
	return 0;
}