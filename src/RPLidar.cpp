
#include <RPLidar.hpp>

#include "RPLidar.hpp"
#include "ReturnDataWrappers.hpp"

using namespace rp_values;
using namespace data_wrappers;

RPLidar::RPLidar(const char *serial_path) : port(serial_path, B115200, 0){}

void RPLidar::print_status(){
	InfoData infoData;
	get_info(&infoData);
	printf("###########################################\n");
	printf("RPLidar Model: A%dM%d\n", infoData.model>>4, infoData.model&0x0F);
	printf("Firmware version: 0x%02X%02X\n", infoData.firmware_major, infoData.firmware_minor);

	HealthData healthData;
	get_health(&healthData);
	printf("Lidar Health: %s\n", healthData.status==rp_values::LidarStatus::LIDAR_OK?"OK":(healthData.status==rp_values::LidarStatus::LIDAR_WARNING?"WARNING":"ERROR"));
	if(healthData.status>0){
		printf("Warning/Error code: %u\n", healthData.error_code);
	}

	SampleRateData sampleRateData;
	get_samplerate(&sampleRateData);
	printf("Scan sampling period(us):%u\nExpress sampling period:%u\n", sampleRateData.scan_sample_rate, sampleRateData.express_sample_rate);
	printf("###########################################\n");
}


/**
 * Sends a packet to the LiDAR
 * @param order that is requested to the LiDAR
 * @param payload of the order, in little endian format
 * @return result of the communication
 */
ComResult RPLidar::send_packet(OrderByte order, const std::vector<uint8_t> &payload) {
	RequestPacket requestPacket(order);
	for(uint8_t data : payload){
		requestPacket.add_payload(data);
	}
	uint8_t timeout=0;
	ComResult result;
	do{
		result=port.send_packet(requestPacket);
		if(timeout>0){
			printf("Error: packet send incomplete, try %d/5\n", ++timeout);
		}
	}while(result==STATUS_ERROR && timeout<5);
	if(timeout>=5){
		printf("Error: could not send packet for order %d", order);
		exit(STATUS_ERROR);
	}
	return STATUS_OK;
}

ComResult RPLidar::get_health(HealthData *health_data) {
	ComResult status=send_packet(GET_HEALTH);
	uint32_t data_len= port.read_descriptor();
	uint8_t* health_array=port.read_data(data_len);
	*health_data=HealthData(health_array);
	delete[] health_array;
	return status;
}

rp_values::ComResult RPLidar::get_info(data_wrappers::InfoData *info_data) {
	ComResult status=send_packet(GET_INFO);
	uint32_t data_len= port.read_descriptor();
	uint8_t* raw_info=port.read_data(data_len);
	*info_data=InfoData(raw_info);
	delete[] raw_info;
	return status;
}

/**
 * Asks the sampling rate to the LiDAR
 * @param sample_rate for express scans (it is actually the period un us)
 * @return result of the communication
 */
ComResult RPLidar::get_samplerate(SampleRateData *sample_rate) {
	send_packet(GET_SAMPLERATE);
	uint32_t data_len= port.read_descriptor();
	uint8_t* sampe_rate_data=port.read_data(data_len);
	*sample_rate=SampleRateData(sampe_rate_data);
	delete[] sampe_rate_data;
	return STATUS_OK;
}

/**
 * Requests the start of the motor to the control module
 * As we do not have any way of knowing if it worked, we have to spam a bit.
 * @return result of the communication
 */
ComResult RPLidar::start_motor() {
	for(int i=0;i<NUMBER_TRIES-1;i++) {
		set_pwm(DEFAULT_MOTOR_PWM);
	}
	return STATUS_OK;
}

/**
 * Requests a stop of the motor to the control module
 * As we do not have any way of knowing if it worked, we have to spam a bit.
 * @return result of the communication
 */
ComResult RPLidar::stop_motor() {
	ComResult status=STATUS_OK;
	for(int i=0;i<NUMBER_TRIES;i++) {
		status=set_pwm(0)==STATUS_OK?STATUS_OK:STATUS_ERROR;
	}
	return status;
}

/**
 * Send a PWM motor command for speed control
 * @param pwm between 0 and 1023 (inclusive)
 * @return result of the communication
 */
ComResult RPLidar::set_pwm(uint16_t pwm) {
	if(pwm>1023){
		pwm=1023;
	}
	uint8_t bytes[2]={(uint8_t)pwm, (uint8_t)(pwm>>8)}; //Little endian
	return send_packet(SET_PWM, {bytes[0],bytes[1]});
}

/**
 * Requests the start of an express scan.
 * A response descriptor should then be read for data packets length,
 * then the data should be read
 * @return result of the communication
 */

ComResult RPLidar::start_express_scan() {
	send_packet(EXPRESS_SCAN, {0,0,0,0,0});
	uint32_t data_size = port.read_descriptor();
	return data_size==DATA_SIZE_EXPRESS_SCAN?ComResult::STATUS_OK:ComResult::STATUS_ERROR;
}


rp_values::ComResult RPLidar::read_scan_data(std::vector<uint8_t> &output_data, bool to_sync) {
	uint8_t* read_data= nullptr;
	uint8_t n_bytes_to_read=DATA_SIZE_EXPRESS_SCAN;
	if(to_sync) {
		read_data = port.read_data(1);
		while (((read_data[0] >> 4) != 0xA)) {
			delete[] read_data;
			read_data = port.read_data(1);
			if (read_data == nullptr) {
				return ComResult::STATUS_ERROR;
			}
		}
		output_data.push_back(read_data[0]);
		delete[] read_data;
		n_bytes_to_read--;
	}
	read_data = port.read_data(n_bytes_to_read);
	if(read_data==nullptr){
		return ComResult::STATUS_ERROR;
	}
	for(uint32_t i=0;i<DATA_SIZE_EXPRESS_SCAN;i++){
		output_data.push_back(read_data[i]);
	}
	delete[] read_data;
	return STATUS_OK;
}


rp_values::ComResult RPLidar::stop_scan() {
	return send_packet(STOP);
}



