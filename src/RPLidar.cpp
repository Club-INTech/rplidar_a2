
#include <RPLidar.hpp>

#include "RPLidar.hpp"


using namespace rp_values;

RPLidar::RPLidar(const char *serial_path) : port(serial_path, 115200, 1){}

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
	uint32_t data_len=port.read_descriptor(GET_HEALTH);
	uint8_t* health=port.read_data(data_len);
	printf("Health:\t");
	for(int i=0;i<data_len;i++){
		printf("%#02x ", health[i]);
	}
	printf("\n");
	delete[] health;
	//TODO: get the health in a useful format
	return 0;
}

int RPLidar::get_info() {
	send_packet(GET_INFO);
	uint32_t data_len=port.read_descriptor(GET_INFO);
	uint8_t* info=port.read_data(data_len);
	printf("Info:\t");
	for(int i=0;i<data_len;i++){
		printf("%#02x ", info[i]);
	}
	printf("\n");
	delete[] info;
	//TODO: get the useful info in a good format
	return 0;
}

int RPLidar::get_samplerate() {
	send_packet(GET_SAMPLERATE);
	uint32_t data_len=port.read_descriptor(GET_SAMPLERATE);
	uint8_t* sprate=port.read_data(data_len);
	printf("Rates:\t");
	for(int i=0;i<data_len;i++){
		printf("%#02x ", sprate[i]);
	}
	printf("\n");
	delete[] sprate;
	//TODO: get the actual samplerate
	return STATUS_OK;
}

int RPLidar::start_motor() {
	for(int i=0;i<NUMBER_TRIES-1;i++) {
		set_pwm(660);
	}
	return STATUS_OK;
}

int RPLidar::stop_motor() {
	for(int i=0;i<NUMBER_TRIES-1;i++) {
		set_pwm(0);
	}
	return STATUS_OK;
}

/**
 * Send a PWM motor command for speed control
 * @param pwm between 0 and 1023 (inclusive)
 * @return
 */
int RPLidar::set_pwm(uint16_t pwm) {
	if(pwm>1023){
		pwm=1023;
	}
	else if(pwm<0){
		pwm=0;
	}
	uint8_t bytes[2]={(uint8_t)pwm, (uint8_t)(pwm>>8)}; //Little endian
	send_packet(SET_PWM, {bytes[0],bytes[1]});
	return 0;
}

void RPLidar::start_express_scan() {
	send_packet(EXPRESS_SCAN, {0,0,0,0,0});
}
