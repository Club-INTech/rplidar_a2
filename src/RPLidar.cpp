//
// Created by tic-tac on 10/4/18.
//

#include <string.h>
#include "RPLidar.hpp"

RPLidar::RPLidar(const char *serial_path) : port(serial_path, 115200, 1){
}

int RPLidar::get_health() {
	return 0;
}

int RPLidar::get_info() {
	return 0;
}

int RPLidar::get_samplerate() {
	return 0;
}

int RPLidar::set_pwm(uint16_t pwm) {
	if(pwm>1023){
		pwm=1023;
	}
	else if(pwm<0){
		pwm=0;
	}
	uint8_t bytes[2]={(uint8_t)pwm, (uint8_t)(pwm>>8)};
	RequestPacket pwmPacket(rp_values::SET_PWM);
	pwmPacket.add_payload(bytes[0]);
	pwmPacket.add_payload(bytes[1]);
	port.send_packet(pwmPacket);
	return 0;
}
