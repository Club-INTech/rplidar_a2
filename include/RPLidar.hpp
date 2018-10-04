//
// Created by tic-tac on 10/4/18.
//

#ifndef RPLIDAR_A2_LIDAR_HPP
#define RPLIDAR_A2_LIDAR_HPP

#include "SerialCommunication.hpp"

#define MAX_PAYLOAD 9

class RPLidar {
	SerialCommunication port;
public:
	RPLidar(const char* serial_path);
	int get_health();
	int get_info();
	int get_samplerate();
	int set_pwm(uint16_t pwm);
};


#endif //RPLIDAR_A2_LIDAR_HPP
