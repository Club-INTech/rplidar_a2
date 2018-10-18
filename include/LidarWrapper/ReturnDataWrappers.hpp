//
// Created by tic-tac on 10/9/18.
//

#ifndef RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
#define RPLIDAR_A2_RETURNDATAWRAPPERS_HPP

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <algorithm>
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


	struct ScanPacket{
		uint8_t quality=0;
		uint8_t new_turn=0;
		float angle=0;
		uint16_t distance=0;

		rp_values::ComResult check_C_bit(const std::vector<uint8_t>& scan_data)
		{
			if((scan_data[1]&1)==0 && ((scan_data[0]&1) == (scan_data[0]&2)>>1)) //if bit C is not 1 and if bit S and /S are not different
			{
				printf("BIT C %d\n", scan_data[1]&1);
				return rp_values::ComResult::STATUS_WRONG_CHECKSUM;
			}
			return rp_values::ComResult::STATUS_OK;
		}

		rp_values::ComResult decode_packet_bytes(const std::vector<uint8_t>& scan_data){
			if(check_C_bit(scan_data)==rp_values::ComResult::STATUS_WRONG_CHECKSUM){
				printf("ERROR: CHECKSUM WRONG ");
				printf("%02X %02X\n",scan_data[0], scan_data[1]);
				return rp_values::ComResult::STATUS_WRONG_CHECKSUM;
			}
			angle=(((scan_data[1]>>1)&0x7F)|(scan_data[2]<<7))/static_cast<float>(64);
			distance=static_cast<uint16_t>(((scan_data[4]<<8) | (scan_data[3]))/ static_cast<float>(4));
			quality= (scan_data[0]>>2)&static_cast<uint8_t>(0x3F);
			return rp_values::ComResult::STATUS_OK;
		}
	};

	struct ExpressScanPacket {
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

		rp_values::ComResult decode_packet_bytes(const std::vector<uint8_t>& raw_bytes) {
			rp_values::ComResult result = check_flags_parity(raw_bytes);
			if(result!=rp_values::ComResult::STATUS_OK){
				return result;
			}
			distances.clear();
			d_angles.clear();
			start_angle = (((raw_bytes[3] & 0x7F) << 8) | (raw_bytes[2])) / static_cast<float>(64);

			for(uint8_t i=4;i<80;i+=5){
				uint16_t d1= static_cast<uint16_t>(((raw_bytes[i]>>2)&0x3F)|(raw_bytes[i+1]<<6));
				uint16_t d2= static_cast<uint16_t>(((raw_bytes[i+2]>>2)&0x3F)|(raw_bytes[i+3]<<6));
				float delta_angle1=((raw_bytes[i+4]&0x0F)|((raw_bytes[i]&1)<<4))/ static_cast<float>(8);
				delta_angle1*=((raw_bytes[i]&2)>>1)==0?1:-1;
				float delta_angle2=(((raw_bytes[i+4]>>4)&0x0F)|((raw_bytes[i+2]&1)<<4))/ static_cast<float>(8);
				delta_angle2*=((raw_bytes[i+2]&2)>>1)==0?1:-1;
				distances.push_back(d1);
				distances.push_back(d2);
				d_angles.push_back(delta_angle1);
				d_angles.push_back(delta_angle2);
			}
			return rp_values::ComResult::STATUS_OK;
		}

		rp_values::ComResult check_flags_parity(const std::vector<uint8_t>& raw_bytes){
			if (((raw_bytes[0] >>4) != 0xA) || ((raw_bytes[1] >> 4) != 0x5)) {
				return rp_values::ComResult::STATUS_WRONG_FLAG;
			}
			uint8_t checksum_received = static_cast<uint8_t>((raw_bytes[0] & 0x0F) | ((raw_bytes[1] & 0x0F) << 4));
			uint8_t checksum_computed = scan_data_checksum(raw_bytes);
			if(checksum_computed!=checksum_received){
				return rp_values::ComResult::STATUS_WRONG_CHECKSUM;
			}
			else{
				return rp_values::ComResult::STATUS_OK;
			}
		}
	};

	struct Measurement{
		float angle=0;
		uint16_t distance=0;
		Measurement(uint16_t d, float a){
			angle=a;
			distance=d;
		}
		bool operator<(const Measurement& other){
			return angle<other.angle;
		}
	};

	struct FullScan{
		std::vector<Measurement> measurements;
		ExpressScanPacket	current_packet; 	//Current express packet data
		ExpressScanPacket next_packet;		//Next express packet data(formula in com. protocol datasheet requires two consecutive scans, cf p23)
		uint8_t measurement_id=32;						//To go through the 32 measurements in each express packet

		void add_measurement(Measurement measurement){
			measurements.push_back(measurement);
		}

		void clear(){
			measurements.clear();
		}

		ssize_t size(){
			return measurements.size();
		}

		Measurement& operator[](uint16_t index){
			return measurements[index];
		}

		std::vector<Measurement>::iterator begin(){
			return measurements.begin();
		}
		std::vector<Measurement>::iterator end(){
			return measurements.end();
		}

		/**
		 * Gets the "measurement_id"th measurement in the current packet, needs the next packet also
		 * @param scan_packet : current express packet
		 * @param next_angle : start angle of next express packet
		 * @param measurement_id : id of the measurement needed
		 * @return Measurement struct (includes distance and angle of the measurement)
		 */
		Measurement get_next_measurement(const data_wrappers::ExpressScanPacket& scan_packet, float next_angle, uint8_t measurement_id) {
			uint16_t distance=scan_packet.distances[measurement_id-1];
			float angle_diff=(next_angle>=scan_packet.start_angle)?(next_angle-scan_packet.start_angle):(360+next_angle-scan_packet.start_angle);
			float angle=fmodf(scan_packet.start_angle+(angle_diff/32.0f)*measurement_id-scan_packet.d_angles[measurement_id-1], 360.0f);
			return {distance, angle};
		}

		//Returns true if there is a new turn
		bool compute_measurements(){
			float last_angle;
			for(measurement_id=0;measurement_id<32;measurement_id++){
				Measurement m=get_next_measurement(current_packet, next_packet.start_angle, measurement_id);
				if(m.angle)
				add_measurement(m);
			}
			return false;
		}
	};
}

#endif //RPLIDAR_A2_RETURNDATAWRAPPERS_HPP
