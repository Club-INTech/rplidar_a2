//
// Created by tic-tac on 10/4/18.
//

#ifndef RPLIDAR_A2_LIDARENUMS_HPP
#define RPLIDAR_A2_LIDARENUMS_HPP
#include <cstdint>

namespace rp_values{
	//Flags used to synchronize with LiDAR
	static constexpr uint8_t START_FLAG=0xA5;
	static constexpr uint8_t START_FLAG_2=0x5A;

	//Number of tries for PWM spam (required as we have no feedback)
	static constexpr uint8_t NUMBER_PWM_TRIES=20;

	//Motor PWM is between 0 and 1023, this is the default speed
	static constexpr uint16_t DEFAULT_MOTOR_PWM=660;

	//Sizes of different output and input packets
	static constexpr uint8_t REQUEST_SIZE=9;
	static constexpr uint8_t DESCRIPTOR_SIZE=7;
	static constexpr uint8_t DATA_SIZE_SCAN=5;
	static constexpr uint8_t DATA_SIZE_EXPRESS_SCAN=84;

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

	enum ComResult{
		STATUS_WRONG_FLAG=-3,
		STATUS_WRONG_CHECKSUM=-2,
		STATUS_ERROR=-1,
		STATUS_OK=0
	};

	//Used for health checking
	enum LidarStatus{
		LIDAR_ERROR=-2,
		LIDAR_WARNING=-1,
		LIDAR_OK=0,
	};
}

#endif //RPLIDAR_A2_LIDARENUMS_HPP
