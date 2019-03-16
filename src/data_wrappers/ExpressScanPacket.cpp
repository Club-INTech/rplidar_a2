//
// Created by tic-tac on 3/15/19.
//

#include "data_wrappers/ExpressScanPacket.hpp"
#include "lidar/LidarEnums.hpp"

ExpressScanPacket::ExpressScanPacket(){
	distances.reserve(32);
	d_angles.reserve(32);
}
uint8_t ExpressScanPacket::scan_data_checksum(const std::vector<uint8_t>& scan_data){
	uint8_t checksum=0;
	for(unsigned long i=2;i<scan_data.size(); i++){
		checksum^=scan_data[i];
	}
	return checksum;
}

int ExpressScanPacket::decode_packet_bytes(const std::vector<uint8_t>& raw_bytes) {
	int result = check_flags_parity(raw_bytes);
	if(result!=rp_values::ComResult::STATUS_OK){
		return result;
	}
	reset();
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

int ExpressScanPacket::check_flags_parity(const std::vector<uint8_t>& raw_bytes){
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

void ExpressScanPacket::reset(){
	d_angles.clear();
	distances.clear();
	start_angle=0;
}
