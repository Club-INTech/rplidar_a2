//
// Created by tic-tac on 3/15/19.
//

#include "data_wrappers/FullScan.hpp"

void FullScan::add_measurement(DataPoint measurement){
	measurements[current_id++]=measurement;
	if(current_id==NBR_DATA){
		current_id=0;
	}
}

void FullScan::clear(){
	measurements.fill({0,0});
}

ssize_t FullScan::size(){
	return measurements.size();
}

DataPoint& FullScan::operator[](uint16_t index){
	return measurements[index];
}

typename std::array<DataPoint,NBR_DATA>::iterator begin(){
	return measurements.begin();
}
typename std::array<DataPoint, NBR_DATA>::iterator end(){
	return measurements.end();
}

/**
 * Gets the "measurement_id"th measurement in the current packet, needs the next packet also
 * @param scan_packet : current express packet
 * @param next_angle : start angle of next express packet
 * @param measurement_id : id of the measurement needed
 * @return Measurement struct (includes distance and angle of the measurement)
 */
DataPoint get_next_measurement(const data_wrappers::ExpressScanPacket& scan_packet, float next_angle, uint8_t measurement_id) {
	uint16_t distance=scan_packet.distances[measurement_id];

	float angle_diff=(next_angle>=scan_packet.start_angle)?(next_angle-scan_packet.start_angle):(360+next_angle-scan_packet.start_angle);
	float angle=fmodf(scan_packet.start_angle+(angle_diff/32.0f)*measurement_id-scan_packet.d_angles[measurement_id-1], 360.0f);
	return {distance, angle};
}

//Returns true if there is a new turn
bool compute_measurements(){
	for(measurement_id=0;measurement_id<32;measurement_id++){
		DataPoint m=get_next_measurement(current_packet, next_packet.start_angle, measurement_id);
		add_measurement(m);
	}
	return false;
}