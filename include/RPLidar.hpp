#ifndef RPLIDAR_A2_LIDAR_HPP
#define RPLIDAR_A2_LIDAR_HPP

#include "SerialCommunication.hpp"

class RPLidar {
	SerialCommunication port;
public:
	RPLidar(const char* serial_path);
	rp_values::ComResult send_packet(rp_values::OrderByte order, const std::vector<uint8_t> &payload={});
	int get_health();
	int get_info();
	rp_values::ComResult get_samplerate(uint16_t *sample_rate);
	rp_values::ComResult set_pwm(uint16_t pwm);
	rp_values::ComResult start_motor();
	rp_values::ComResult stop_motor();

	rp_values::ComResult start_express_scan();
};

#endif //RPLIDAR_A2_LIDAR_HPP
