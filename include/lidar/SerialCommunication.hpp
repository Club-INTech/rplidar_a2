/*
 *  Class implementing Serial Communication for RPLidar A2's protocol.
 */

#ifndef RPLIDAR_A2_SERIALCOM_HPP
#define RPLIDAR_A2_SERIALCOM_HPP

#include <string>
#include <termios.h>
#include "data_wrappers/RequestPacket.hpp"

class SerialCommunication {
	int serial_fd=0;
	bool is_open = true;
	std::string path;
	uint8_t data[rp_values::REQUEST_SIZE]={0};
	void setDTR(bool enable);
	int set_interface_attribs(speed_t speed, int parity);
	void set_blocking(bool should_block, uint8_t timeout);
public:
	SerialCommunication()=default;
	~SerialCommunication();
	bool init_port(const char *filePath, speed_t baudrate, int parity);
	int close_port();
	rp_values::ComResult send_packet(const RequestPacket &packet);
	uint32_t read_descriptor();
	uint8_t* read_data(uint32_t num_bytes);
	uint8_t read_byte();
	void flush();
};


#endif //RPLIDAR_A2_SERIALCOM_HPP
