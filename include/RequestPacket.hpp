//
// Created by tic-tac on 10/1/18.
//

#ifndef RPLIDAR_A2_SERIALORDER_HPP
#define RPLIDAR_A2_SERIALORDER_HPP

#include <vector>

#include "LidarEnums.hpp"


struct RequestPacket{
	uint8_t order;					// Ordre à envoyer
	uint8_t payload_size;	// Nombre de paramètres à envoyer
	uint8_t checksum=0;	// Recalculé à chaque ajout de valeur de payload
	std::vector<uint8_t> payload;	// Arguments de l'ordre

	RequestPacket(uint8_t p_order):order(p_order){
		payload.clear();
		//Only SET_PWM and EXPRESS_SCAN require a payload
		switch(p_order){
			case rp_values::SET_PWM: payload_size=2;break;
			case rp_values::EXPRESS_SCAN: payload_size=5;break;
			default: payload_size=0;break;
		}
		checksum^=rp_values::START_FLAG^order^payload_size;
	}

	bool add_payload(uint8_t value){
		if(payload.size()<payload_size){
			payload.push_back(value);
			checksum^=value;
			return true;
		}
		return false;
	}

	uint8_t get_packet(uint8_t* output_packet)const{
		output_packet[0]=rp_values::START_FLAG;
		output_packet[1]=order;
		if(payload_size>0){
			output_packet[2]=payload_size;
			for(int i=0;i<payload_size;i++){
				output_packet[i+3]=payload[i];
			}
			output_packet[payload_size+3]=checksum;
		}
		return sizeof(uint8_t)*(2+payload_size+(payload_size>0?2:0));
	}
};

#endif //RPLIDAR_A2_SERIALORDER_HPP
