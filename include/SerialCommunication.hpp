/*
 *  Class implementing Serial Communication for RPLidar A2's protocol.
 */

#ifndef RPLIDAR_A2_SERIALCOM_HPP
#define RPLIDAR_A2_SERIALCOM_HPP

#include <string>
#include <termios.h>
#include "RequestPacket.hpp"

class SerialCommunication {
	int serial_fd;
	uint8_t data[9];
	void setDTR(bool enable);
	int set_interface_attribs(int speed, int parity);
	void set_blocking(bool should_block);
public:
	SerialCommunication(const char* filePath, speed_t baudrate, int parity);
	void send_packet(const RequestPacket& packet);
};


#endif //RPLIDAR_A2_SERIALCOM_HPP
