#ifndef RPLIDAR_A2_LIDAR_HPP
#define RPLIDAR_A2_LIDAR_HPP

#include "SerialCommunication.hpp"
#include "ReturnDataWrappers.hpp"

class RPLidar {
	SerialCommunication port;
public:
	RPLidar(const char* serial_path);
	rp_values::ComResult send_packet(rp_values::OrderByte order, const std::vector<uint8_t> &payload={});
	int get_health(data_wrappers::HealthData *health_data);
	int get_info();
	rp_values::ComResult get_samplerate(data_wrappers::SampleRateData *sample_rate);
	rp_values::ComResult set_pwm(uint16_t pwm);
	rp_values::ComResult start_motor();
	rp_values::ComResult stop_motor();

	rp_values::ComResult start_express_scan();
	rp_values::ComResult stop_scan();
};

#endif //RPLIDAR_A2_LIDAR_HPP
