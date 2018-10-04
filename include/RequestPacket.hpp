//
// Created by tic-tac on 10/1/18.
//

#ifndef RPLIDARINTECHLIB_SERIALORDER_HPP
#define RPLIDARINTECHLIB_SERIALORDER_HPP

#include <vector>
#include <stdint.h>

static constexpr uint8_t START_FLAG=0xA5;

enum OrderByte{
	STOP 							= (uint8_t) 0x25,
	RESET 						= (uint8_t) 0x40,
	SCAN 							= (uint8_t) 0x20,
	EXPRESS_SCAN 		= (uint8_t) 0x82,
	FORCE_SCAN 			= (uint8_t) 0x21,
	SET_PWM 					= (uint8_t) 0xF0,
	GET_INFO 					= (uint8_t) 0x50,
	GET_HEALTH 			= (uint8_t) 0x52,
	GET_SAMPLERATE 	= (uint8_t) 0x59
};

struct RequestPacket{
	uint8_t order;					// Ordre à envoyer
	uint8_t payload_size;	// Nombre de paramètres à envoyer
	uint8_t checksum=0;	// Recalculé à chaque ajout de valeur de payload
	std::vector<uint8_t> payload;	// Arguments de l'ordre

	RequestPacket(uint8_t p_order):order(p_order){
		payload.clear();
		switch(p_order){
			case SET_PWM: payload_size=2;break;
			case EXPRESS_SCAN: payload_size=5;break;
			default: payload_size=0;break;
		}
		checksum^=START_FLAG^order^payload_size;
	}

	bool add_payload(uint8_t value){
		if(payload.size()<payload_size){
			payload.push_back(value);
			checksum^=value;
			return true;
		}
		return false;
	}

	void get_packet(uint8_t* output_packet){
		output_packet[0]=START_FLAG;
		output_packet[1]=order;
		if(payload_size>0){
			for(int i=0;i<payload_size;i++){
				output_packet[i+2]=payload[i];
			}
			output_packet[payload_size+2]=checksum;
		}
	}
};

#endif //RPLIDARINTECHLIB_SERIALORDER_HPP
