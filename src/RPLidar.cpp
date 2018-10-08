
#include <RPLidar.hpp>

#include "RPLidar.hpp"


using namespace rp_values;

RPLidar::RPLidar(const char *serial_path) : port(serial_path, B115200, 0){}

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

int RPLidar::get_health() {
	send_packet(GET_HEALTH);
	uint32_t data_len= port.read_descriptor();
	uint8_t* health=port.read_data(data_len);
	printf("Health:\t");
	for(uint32_t i=0;i<data_len;i++){
		printf("%#02x ", health[i]);
	}
	printf("\n");
	delete[] health;
	//TODO: get the health in a useful format
	return 0;
}

int RPLidar::get_info() {
	send_packet(GET_INFO);
	uint32_t data_len= port.read_descriptor();
	uint8_t* info=port.read_data(data_len);
	printf("Info:\t");
	for(uint32_t i=0;i<data_len;i++){
		printf("%#02x ", info[i]);
	}
	printf("\n");
	delete[] info;
	//TODO: get the useful info in a good format
	return 0;
}

/**
 * Asks the sampling rates to the LiDAR
 * @param sample_rate for express scans (it is actually the period un us)
 * @return result of the communication
 */
ComResult RPLidar::get_samplerate(uint16_t *sample_rate) {
	send_packet(GET_SAMPLERATE);
	uint32_t data_len= port.read_descriptor();
	uint8_t* sampe_rate_data=port.read_data(data_len);
	*sample_rate = (sampe_rate_data[3]<<8) | (sampe_rate_data[2]);
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
		set_pwm(660);
	}
	return STATUS_OK;
}

/**
 * Requests a stop of the motor to the control module
 * As we do not have any way of knowing if it worked, we have to spam a bit.
 * @return result of the communication
 */
ComResult RPLidar::stop_motor() {
	for(int i=0;i<NUMBER_TRIES-1;i++) {
		set_pwm(0);
	}
	return set_pwm(0);
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
	return send_packet(EXPRESS_SCAN, {0,0,0,0,0});
}
