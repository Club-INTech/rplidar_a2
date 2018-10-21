//
// Created by tic-tac on 10/19/18.
//

#ifndef RPLIDAR_A2_DATASOCKET_HPP
#define RPLIDAR_A2_DATASOCKET_HPP

#include <iostream>
#include <sys/socket.h>
#include <libnet.h>
#include <fcntl.h>
#include <netinet/in.h>

#include <LidarWrapper/ReturnDataWrappers.hpp>

class DataSocket {
public:
	int server_socket=0;
	int client_socket=0;
	struct sockaddr_in server_address;
public:
	DataSocket(const char *address_string, uint16_t server_port);
	bool accept_client();
	int send_scan(data_wrappers::FullScan &scan);
};


#endif //RPLIDAR_A2_DATASOCKET_HPP
