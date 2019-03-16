//
// Created by tic-tac on 3/15/19.
//

#ifndef RPLIDAR_A2_FULLSCAN_HPP
#define RPLIDAR_A2_FULLSCAN_HPP

#include <array>

template<int NBR_DATA>
class FullScan{
	std::array<DataPoint, NBR_DATA> measurements;
	ExpressScanPacket	current_packet; 	//Current express packet data
	ExpressScanPacket next_packet;		//Next express packet data(formula in com. protocol datasheet requires two consecutive scans, cf p23)
	uint16_t current_id=0;
	uint8_t measurement_id=32;						//To go through the 32 measurements in each express packet

	void add_measurement(DataPoint measurement);

	void clear();

	ssize_t size();

	DataPoint& operator[](uint16_t index);

	typename std::array<DataPoint,NBR_DATA>::iterator begin();
	typename std::array<DataPoint, NBR_DATA>::iterator end();

	/**
	 * Gets the "measurement_id"th measurement in the current packet, needs the next packet also
	 * @param scan_packet : current express packet
	 * @param next_angle : start angle of next express packet
	 * @param measurement_id : id of the measurement needed
	 * @return Measurement struct (includes distance and angle of the measurement)
	 */
	DataPoint get_next_measurement(const data_wrappers::ExpressScanPacket& scan_packet, float next_angle, uint8_t measurement_id) ;

	//Returns true if there is a new turn
	bool compute_measurements();
};


#endif //RPLIDAR_A2_FULLSCAN_HPP
