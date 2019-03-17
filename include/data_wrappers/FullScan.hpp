//
// Created by tic-tac on 3/15/19.
//

#ifndef RPLIDAR_A2_FULLSCAN_HPP
#define RPLIDAR_A2_FULLSCAN_HPP

#include <vector>
#include "data_wrappers/ReturnDataWrappers.hpp"
#include "data_wrappers/ExpressScanPacket.hpp"

struct FullScan{

	constexpr static uint8_t CFG_NB_PACKET_PER_TURN=11;
	constexpr static uint8_t CFG_NB_SAMPLE_PER_PACKET=32;

	std::vector<std::pair<float, uint16_t>> measurements;
	ExpressScanPacket	current_packet; 	//Current express packet data
	ExpressScanPacket next_packet;		//Next express packet data(formula in com. protocol datasheet requires two consecutive scans, cf p23)
	uint16_t current_id=0;
	uint8_t measurement_id=CFG_NB_SAMPLE_PER_PACKET;						//To go through the 32 measurements in each express packet

	FullScan(){
	    measurements.reserve(CFG_NB_PACKET_PER_TURN*CFG_NB_SAMPLE_PER_PACKET);
        for(uint16_t i = 0; i < measurements.capacity();i++){
            measurements.emplace_back(std::pair<float, uint16_t>(0,0));
        }
    }
	void add_measurement(std::pair<float, uint16_t> measurement){
		measurements[current_id].first=measurement.first;
		measurements[current_id++].second=measurement.second;
		if(current_id==CFG_NB_SAMPLE_PER_PACKET*CFG_NB_PACKET_PER_TURN){
			current_id=0;
		}
	}

	void clear(){
		std::fill(measurements.begin(), measurements.end(), std::pair<float, uint16_t>(0,0));
	}

	ssize_t size(){
		return measurements.size();
	}

	std::pair<float, uint16_t>& operator[](uint16_t index){
		return measurements[index];
	}

	typename std::vector<std::pair<float, uint16_t>>::iterator begin(){
		return measurements.begin();
	}
	typename std::vector<std::pair<float, uint16_t>>::iterator end(){
		return measurements.end();
	}

	/**
	 * Gets the "measurement_id"th measurement in the current packet, needs the next packet also
	 * @param scan_packet : current express packet
	 * @param next_angle : start angle of next express packet
	 * @param measurement_id : id of the measurement needed
	 * @return Measurement struct (includes distance and angle of the measurement)
	 */
	std::pair<float, uint16_t> get_next_measurement(const ExpressScanPacket& scan_packet, float next_angle, uint8_t measurement_id){
		uint16_t distance=scan_packet.distances[measurement_id];

		//Formulas for angle from slamtec protocol manual
		float angle_diff=(next_angle>=scan_packet.start_angle)?(next_angle-scan_packet.start_angle):(360+next_angle-scan_packet.start_angle);
		float angle=fmodf(scan_packet.start_angle+(angle_diff/32.0f)*measurement_id-scan_packet.d_angles[measurement_id-1], 360.0f);

		return(std::pair<float, uint16_t>(angle, distance));
	}

	//Returns true if there is a new turn
	bool compute_measurements(){
		for(measurement_id=0;measurement_id<CFG_NB_SAMPLE_PER_PACKET;measurement_id++){
			std::pair<float, uint16_t> m=get_next_measurement(current_packet, next_packet.start_angle, measurement_id);
			add_measurement(m);
		}
		return false;
	}
};


#endif //RPLIDAR_A2_FULLSCAN_HPP
