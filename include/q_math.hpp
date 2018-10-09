//
// Created by tic-tac on 10/9/18.
//

#ifndef RPLIDAR_A2_Q_MATH_HPP
#define RPLIDAR_A2_Q_MATH_HPP
#include <cstdint>
#include <cmath>
uint16_t float_to_q(float value, uint8_t Q){
	return value*powf(2, Q);
}

float q_to_float(uint16_t value, uint8_t Q){
	return value*powf(2,-Q);
}

uint16_t q_div(uint16_t a, uint16_t b, uint8_t Q ){
	uint32_t temp = (uint32_t)a<<Q;
	temp+=b/2;
	return (uint16_t)(temp/b);
}

#endif //RPLIDAR_A2_Q_MATH_HPP
