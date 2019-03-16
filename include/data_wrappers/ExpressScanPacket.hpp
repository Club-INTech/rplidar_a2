//
// Created by tic-tac on 3/15/19.
//

#ifndef RPLIDAR_A2_EXPRESSSCANPACKET_HPP
#define RPLIDAR_A2_EXPRESSSCANPACKET_HPP

#include <vector>
#include <cstdint>

struct ExpressScanPacket {
	std::vector<uint16_t> distances;        //measurement distance for each value in the packet
	std::vector<float> d_angles;            //delta angle for each value in the packet
	float start_angle=0;                                //Start angle of the packet

	ExpressScanPacket();

	uint8_t scan_data_checksum(const std::vector<uint8_t>& scan_data);
	int decode_packet_bytes(const std::vector<uint8_t>& raw_bytes);
	int check_flags_parity(const std::vector<uint8_t>& raw_bytes);
	void reset();
};

#endif //RPLIDAR_A2_EXPRESSSCANPACKET_HPP
