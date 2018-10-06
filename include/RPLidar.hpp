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
	int get_samplerate();
	int set_pwm(uint16_t pwm);
	int start_motor();
	int stop_motor();

	void start_express_scan();
};

#endif //RPLIDAR_A2_LIDAR_HPP
