//
// Created by tic-tac on 10/1/18.
//

#ifndef RPLIDARINTECHLIB_SERIALCOM_HPP
#define RPLIDARINTECHLIB_SERIALCOM_HPP

#include <termios.h>
#include <string>
#include "RequestPacket.hpp"

class SerialCom {
	uint8_t serial_fd;
	termios tty;

public:
	SerialCom(const std::string& filePath, uint16_t baudrate);
	void send_packet(RequestPacket packet);
};


#endif //RPLIDARINTECHLIB_SERIALCOM_HPP
