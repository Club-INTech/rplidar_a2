//
// Created by tic-tac on 10/9/18.
//

#ifndef RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
#define RPLIDAR_A2_RETURNDATAWRAPPERS_HPP

#include <cstdint>
#include "LidarEnums.hpp"

namespace data_wrappers {

	struct SampleRateData {
		uint16_t scan_sample_rate;
		uint16_t express_sample_rate;

		SampleRateData() {
			scan_sample_rate = 0;
			express_sample_rate = 0;
		}

		/**
		 * Constructs the sample rate data from a 4 byte data array
		 * @param raw_data pointer to a 4 byte data array
		 */
		SampleRateData(const uint8_t *raw_data) {
			scan_sample_rate = (raw_data[1] << 8) | (raw_data[0]);
			express_sample_rate = (raw_data[3] << 8) | (raw_data[2]);
		}
	};


	struct HealthData{
		rp_values::LidarStatus status;
		uint16_t error_code;

		HealthData(){
			status=rp_values::LidarStatus::LIDAR_OK;
			error_code=0;
		}

		/**
		 * Constructs the health data from a 3 byte data array
		 * @param raw_data pointer to a 3 byte array
		 */
		HealthData(const uint8_t* raw_data){
			switch(raw_data[0]){
				case 0: status=rp_values::LidarStatus::LIDAR_OK;break;
				case 1: status=rp_values::LidarStatus::LIDAR_WARNING;break;
				case 2: status=rp_values::LidarStatus::LIDAR_ERROR;break;
				default:status=rp_values::LidarStatus::LIDAR_OK;break;
			}
			error_code=(raw_data[2]<<8)|(raw_data[1]);
		}
	};
}

#endif //RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
