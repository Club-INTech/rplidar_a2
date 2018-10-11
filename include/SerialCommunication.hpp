/*
 *  Class implementing Serial Communication for RPLidar A2's protocol.
 */

#ifndef RPLIDAR_A2_SERIALCOM_HPP
#define RPLIDAR_A2_SERIALCOM_HPP

#include <string>
#include <termios.h>
#include "RequestPacket.hpp"

class SerialCommunication {
public:
	int serial_fd;
	std::string path;
	uint8_t data[9];
	void setDTR(bool enable);
	int set_interface_attribs(int speed, int parity);
	void set_blocking(bool should_block);
public:
	SerialCommunication(const char* filePath, speed_t baudrate, int parity);
	rp_values::ComResult send_packet(const RequestPacket &packet);

	uint8_t read_byte();
	uint32_t read_descriptor();
	uint8_t* read_data(uint32_t num_bytes);
};


#endif //RPLIDAR_A2_SERIALCOM_HPP
