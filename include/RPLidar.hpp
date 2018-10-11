#ifndef RPLIDAR_A2_LIDAR_HPP
#define RPLIDAR_A2_LIDAR_HPP

#include "SerialCommunication.hpp"
#include "ReturnDataWrappers.hpp"
#include <sys/time.h>

class RPLidar {
public:
	SerialCommunication port;

	RPLidar(const char* serial_path);

	void print_status();
	rp_values::ComResult send_packet(rp_values::OrderByte order, const std::vector<uint8_t> &payload={});
	rp_values::ComResult get_health(data_wrappers::HealthData *health_data);
	rp_values::ComResult get_info(data_wrappers::InfoData *info_data);
	rp_values::ComResult get_samplerate(data_wrappers::SampleRateData *sample_rate);
	rp_values::ComResult set_pwm(uint16_t pwm);
	rp_values::ComResult start_motor();
	rp_values::ComResult stop_motor();

	data_wrappers::Measurement get_next_measurement(data_wrappers::ExpressScanPacket& scan_packet, float next_angle, uint8_t measurement_id);
	rp_values::ComResult start_express_scan();
	rp_values::ComResult read_scan_data(std::vector<uint8_t> &output_data, uint8_t size, bool to_sync = false);
	rp_values::ComResult stop_scan();

	rp_values::ComResult start_scan();

	void process_express_scans();

	void process_regular_scans();
};

#endif //RPLIDAR_A2_LIDAR_HPP
