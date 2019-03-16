//
// Created by tic-tac on 10/9/18.
//

#ifndef RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
#define RPLIDAR_A2_RETURNDATAWRAPPERS_HPP

#include <math.h>
#include <cstdint>
#include <cstdio>
#include <array>
#include <vector>
#include <algorithm>
#include <iostream>
#include "lidar/LidarEnums.hpp"


namespace data_wrappers {
//Extracts sample rate data from packet
	struct SampleRateData {
		uint16_t scan_sample_rate = 0;
		uint16_t express_sample_rate = 0;

		SampleRateData() = default;

		/**
         * Constructs the sample rate data from a 4 byte data array
         * @param raw_data pointer to a 4 byte data array
         */
		SampleRateData(const uint8_t *raw_data) {
			scan_sample_rate = (raw_data[1] << 8) | (raw_data[0]);
			express_sample_rate = (raw_data[3] << 8) | (raw_data[2]);
		}
	};

//Extracts health data from packet
	struct HealthData {
		rp_values::LidarStatus status = rp_values::LidarStatus::LIDAR_OK;
		uint16_t error_code = 0;

		HealthData() = default;

		/**
         * Constructs the health data from a 3 byte data array
         * @param raw_data pointer to a 3 byte array
         */
		HealthData(const uint8_t *raw_data) {
			switch (raw_data[0]) {
				case 0:
					status = rp_values::LidarStatus::LIDAR_OK;
					break;
				case 1:
					status = rp_values::LidarStatus::LIDAR_WARNING;
					break;
				case 2:
					status = rp_values::LidarStatus::LIDAR_ERROR;
					break;
				default:
					status = rp_values::LidarStatus::LIDAR_OK;
					break;
			}
			error_code = (raw_data[2] << 8) | (raw_data[1]);
		}
	};

//Extracts misc. info about the lidar data from packet
	struct InfoData {
		uint8_t model = 0;
		uint8_t firmware_minor = 0;
		uint8_t firmware_major = 0;
		uint8_t hardware = 0;
		uint8_t serial_number[16] = {0};

		InfoData() = default;

		/**
         * Constructs the lidar info data from a 20 byte data array
         * @param raw_data pointer to a 20 byte arraya
         */
		InfoData(const uint8_t *raw_data) {
			model = raw_data[0];
			firmware_minor = raw_data[1];
			firmware_major = raw_data[2];
			hardware = raw_data[3];
			for (int i = 0; i < 16; i++) {
				serial_number[i] = raw_data[i + 4];
			}
		}
	};
}

#endif //RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
