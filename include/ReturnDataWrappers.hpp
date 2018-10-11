//
// Created by tic-tac on 10/9/18.
//

#ifndef RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
#define RPLIDAR_A2_RETURNDATAWRAPPERS_HPP

#include <cstdint>
#include <cstdio>
#include <vector>
#include "LidarEnums.hpp"

namespace data_wrappers {
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

	struct ExpressPacket {
		std::vector<uint16_t> distances;        //measurement distance for each value in the packet
		std::vector<float> d_angles;            //delta angle for each value in the packet
		float start_angle=0;                                //Start angle of the packet
		uint8_t current_measurement=0;

		uint8_t scan_data_checksum(const std::vector<uint8_t>& scan_data){
			uint8_t checksum=0;
			for(unsigned long i=2;i<scan_data.size(); i++){
				checksum^=scan_data[i];
			}
			return checksum;
		}

		bool decode_packet_bytes(const std::vector<uint8_t>& raw_bytes) {
			if(!check_flags_parity(raw_bytes)){
				return false;
			}
			start_angle=(((raw_bytes[3]&0x7F)<<8)|(raw_bytes[2]))/ static_cast<float>(64);
			return true;
		}

		bool check_flags_parity(const std::vector<uint8_t>& raw_bytes){
			if (((raw_bytes[0] >>4) != 0xA) || ((raw_bytes[1] >> 4) != 0x5)) {
				printf("WRONG FLAG 0x%02X 0x%02X\n", raw_bytes[0], raw_bytes[1]);
				return false; //Wrong flag: ignore the value
			}
			uint8_t checksum_received = static_cast<uint8_t>((raw_bytes[0] & 0x0F) | ((raw_bytes[1] & 0x0F) << 4));
			uint8_t checksum_computed = scan_data_checksum(raw_bytes);
			if(checksum_computed!=checksum_received){
				printf("WRONG FLAG\n");
			}
			return checksum_received == checksum_computed;
		}
	};
}

#endif //RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
