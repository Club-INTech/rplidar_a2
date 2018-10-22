#ifndef RPLIDAR_A2_LIDAR_HPP
#define RPLIDAR_A2_LIDAR_HPP

#include "SerialCommunication.hpp"
#include "ReturnDataWrappers.hpp"
#include <sys/time.h>

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
	rp_values::ComResult read_scan_data(std::vector<uint8_t> &output_data, uint8_t size, bool to_sync = false);
	rp_values::ComResult stop_scan();

	int8_t check_new_turn(float next_angle, data_wrappers::FullScan &current_scan);
	bool process_express_scans(data_wrappers::FullScan &current_scan, bool debug=false);

	void print_scan(data_wrappers::FullScan scan);
};

#endif //RPLIDAR_A2_LIDAR_HPP
