//
// Created by tic-tac on 10/4/18.
//

#ifndef RPLIDAR_A2_LIDARENUMS_HPP
#define RPLIDAR_A2_LIDARENUMS_HPP
#include <stdint.h>

namespace rp_values{
	static constexpr uint8_t START_FLAG=0xA5;
	static constexpr uint8_t MAX_PAYLOAD=9;
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
}

#endif //RPLIDAR_A2_LIDARENUMS_HPP
