//
// Created by tic-tac on 10/1/18.
//

#ifndef RPLIDAR_A2_SERIALORDER_HPP
#define RPLIDAR_A2_SERIALORDER_HPP

#include <vector>
#include "lidar/LidarEnums.hpp"


struct RequestPacket{
	rp_values::OrderByte order;					// Ordre à envoyer
	uint8_t payload_size;	// Nombre de paramètres à envoyer
	uint8_t checksum=0;	// Recalculé à chaque ajout de valeur de payload
	std::vector<uint8_t> payload;	// Arguments de l'ordre

    RequestPacket(rp_values::OrderByte p_order);
    bool add_payload(uint8_t value);
    uint8_t get_packet(uint8_t* output_packet)const;
};

#endif //RPLIDAR_A2_SERIALORDER_HPP
