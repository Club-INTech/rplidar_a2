#include "RPLidar.hpp"
#include <csignal>
#include <RPLidar.hpp>
#include <cmath>

#include "ReturnDataWrappers.hpp"

using namespace rp_values;
using namespace data_wrappers;

bool running=true;

double msecs()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	return (double) tv.tv_usec / 1000 + tv.tv_sec * 1000;
}
void signal_handler(int signo){
	if(signo==SIGTERM || signo==SIGINT)
		running = false;
}


RPLidar::RPLidar(const char *serial_path) : port(serial_path, B115200, 0){
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);
	running=true;
}

void RPLidar::print_status(){
	InfoData infoData;
	get_info(&infoData);
	printf("#########################################\n");
	printf("#\t\tRPLidar Model: A%dM%d\t\t\t\t#\n", infoData.model>>4, infoData.model&0x0F);
	printf("#\t\tFirmware version: 0x%02X%02X\t\t#\n", infoData.firmware_major, infoData.firmware_minor);

	HealthData healthData;
	get_health(&healthData);
	printf("#\t\tLidar Health: %s\t\t\t\t#\n", healthData.status==rp_values::LidarStatus::LIDAR_OK?"OK":(healthData.status==rp_values::LidarStatus::LIDAR_WARNING?"WARNING":"ERROR"));
	if(healthData.status>0){
		printf("Warning/Error code: %u\n", healthData.error_code);
	}

	SampleRateData sampleRateData;
	get_samplerate(&sampleRateData);
	printf("#\t\tScan sampling period(us):%u\t#\n#\t\tExpress sampling period:%u\t\t#\n", sampleRateData.scan_sample_rate, sampleRateData.express_sample_rate);
	printf("#########################################\n");
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


rp_values::ComResult RPLidar::read_scan_data(std::vector<uint8_t> &output_data, uint8_t size, bool to_sync) {
	uint8_t* read_data= nullptr;
	uint8_t n_bytes_to_read=size;
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
	for(uint32_t i=0;i<n_bytes_to_read;i++){
		output_data.push_back(read_data[i]);
	}
	delete[] read_data;
	return STATUS_OK;
}


rp_values::ComResult RPLidar::stop_scan() {
	return send_packet(STOP);
}

ComResult RPLidar::start_scan() {
	send_packet(SCAN);
	uint32_t data_size = port.read_descriptor();
	return data_size==DATA_SIZE_SCAN?ComResult::STATUS_OK:ComResult::STATUS_ERROR;

}

float AngleDiff(float angle1, float angle2){
	if(angle2>=angle1){
		return angle2-angle1;
	}
	return 360+angle2-angle1;
}

data_wrappers::Measurement
RPLidar::get_next_measurement(data_wrappers::ExpressScanPacket &scan_packet, float next_angle, uint8_t measurement_id) {
	uint16_t distance=scan_packet.distances[measurement_id-1];
	float angle=fmodf(scan_packet.start_angle+(AngleDiff(scan_packet.start_angle, next_angle)/32.0f)*measurement_id-scan_packet.d_angles[measurement_id-1], 360.0f);
	return {distance, angle};
}

void RPLidar::process_express_scans() {
	bool wrong_flag=false;
	std::vector<uint8_t> read_buffer;
	ExpressScanPacket packet_current;
	ExpressScanPacket packet_next;
	uint8_t measurement_id=32;
	// Need two packets for angle computing: current and last (cf formula in p23 of com. protocol documentation)
	double start_time = msecs();
	while(running) {
		if(measurement_id==32) {
			measurement_id = 0;
			if (packet_next.distances.empty()) {
				read_buffer.clear();
				read_scan_data(read_buffer, DATA_SIZE_EXPRESS_SCAN, wrong_flag);
				rp_values::ComResult result = packet_next.decode_packet_bytes(read_buffer);
				if (result == rp_values::ComResult::STATUS_WRONG_FLAG) {
					printf("WRONG FLAGS, WILL SYNC BACK\n");
					wrong_flag = true;
					continue; //Couldn't decode an Express packet (wrong flags or checksum)
				} else if (result != rp_values::ComResult::STATUS_OK) {
					printf("WRONG CHECKSUM OR COM ERROR, IGNORE PACKET\n");
					continue;
				}
			}
			packet_current = packet_next;
			read_buffer.clear();
			read_scan_data(read_buffer, DATA_SIZE_EXPRESS_SCAN, wrong_flag);
			rp_values::ComResult result = packet_next.decode_packet_bytes(read_buffer);
			if (result == rp_values::ComResult::STATUS_WRONG_FLAG) {
				printf("WRONG FLAGS, WILL SYNC BACK\n");
				wrong_flag = true;
				continue; //Couldn't decode an Express packet (wrong flags or checksum)
			} else if (result != rp_values::ComResult::STATUS_OK) {
				printf("WRONG CHECKSUM OR COM ERROR, IGNORE PACKET\n");
				continue;
			}
			wrong_flag = false;
		}
//		for(uint8_t data_byte : read_buffer){
//			printf("0x%02X ",data_byte);
//		}
//		printf("\n");
		measurement_id++;
		Measurement measurement=get_next_measurement(packet_current, packet_next.start_angle, measurement_id);
		double duration= msecs()-start_time;
		start_time= msecs();
		printf("d %u ang %f\n", measurement.distance, measurement.angle);
		printf("Delta_t = %dms\n", static_cast<uint32_t>(duration));
	}
}

void RPLidar::process_regular_scans() {
	std::vector<uint8_t> read_buffer;
	ScanPacket packet_current;

	// Need two packets for angle computing: current and last (cf formula in p23 of com. protocol documentation)
	double start_time = msecs();
	while(running) {
		read_buffer.clear();
		read_scan_data(read_buffer, DATA_SIZE_SCAN);
		for(uint8_t data_byte : read_buffer){
			printf("0x%02X ",data_byte);
		}
		printf("\n");
		rp_values::ComResult result=packet_current.decode_packet_bytes(read_buffer);
		if(result != rp_values::ComResult::STATUS_OK){
			printf("WRONG CHECKSUM OR COM ERROR, IGNORE PACKET\n");
			continue;
		}

		double duration= msecs()-start_time;
		start_time= msecs();
		printf("New Packet: Angle=%f, distance=%u\n", packet_current.angle, packet_current.distance);
		printf("Delta_t = %dms\n", static_cast<uint32_t>(duration));
	}
}
