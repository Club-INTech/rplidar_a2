#ifndef RPLIDAR_A2_LIDAR_HPP
#define RPLIDAR_A2_LIDAR_HPP

#include "SerialCommunication.hpp"
#include "ReturnDataWrappers.hpp"

uint8_t scan_data_checksum(const std::vector<uint8_t>& scan_data);

class RPLidar {
	SerialCommunication port;
public:
	RPLidar(const char* serial_path);

	void print_status();
	rp_values::ComResult send_packet(rp_values::OrderByte order, const std::vector<uint8_t> &payload={});
	rp_values::ComResult get_health(data_wrappers::HealthData *health_data);
	rp_values::ComResult get_info(data_wrappers::InfoData *info_data);
	rp_values::ComResult get_samplerate(data_wrappers::SampleRateData *sample_rate);
	rp_values::ComResult set_pwm(uint16_t pwm);
	rp_values::ComResult start_motor();
	rp_values::ComResult stop_motor();

	rp_values::ComResult start_express_scan();
	rp_values::ComResult read_scan_data(std::vector<uint8_t>& output_data);
	rp_values::ComResult stop_scan();
};

#endif //RPLIDAR_A2_LIDAR_HPP
