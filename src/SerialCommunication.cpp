//
// Created by tic-tac on 10/1/18.
//

#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <SerialCommunication.hpp>


#include "SerialCommunication.hpp"

using namespace rp_values;

SerialCommunication::SerialCommunication(const char *filePath, speed_t baudrate, int parity) {
	serial_fd=open(filePath, O_RDWR);
	set_interface_attribs(baudrate, parity); 					//8N1 at 115200
	set_blocking(true);												//Non Blocking communication
	setDTR(false);														//DTR wire is used for PWM control
	memset(data, 0, rp_values::MAX_PAYLOAD);	//Initialize data to 0
}


/**
 * Enables or disables the DTR control signal
 * @param enable state
 */
void SerialCommunication::setDTR(bool enable) {
	int flags;
	ioctl(serial_fd, TIOCMGET, &flags);
	enable ? (flags |= TIOCM_DTR) : (flags &= ~TIOCM_DTR);
	ioctl(serial_fd, TIOCMSET, &flags);
}


/**
 * Sets the file descriptor interface attributes for Serial communication
 * @param speed : the baudrate(of type speed_t) (usually B115200)
 * @param parity : parity of the communication(usually 0)
 * @return
 */
int SerialCommunication::set_interface_attribs(int speed, int parity) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (serial_fd, &tty) != 0)
	{
		perror ("error from tcgetattr");
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // ignore break signal
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 10;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity

	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (serial_fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}


/**
 * Sets blocking state of the file descriptor interface
 * @param should_block
 */
void SerialCommunication::set_blocking(bool should_block) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (serial_fd, &tty) != 0)
	{
		printf ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 2;            	// 0.5 seconds read timeout

	if (tcsetattr (serial_fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes", errno);
}


/**
 * Sends data from a RequestPacket reference
 * @param packet
 */
ComResult SerialCommunication::send_packet(const RequestPacket &packet) {
	memset(data, 0, MAX_PAYLOAD);
	uint8_t size=packet.get_packet(data);
	ssize_t written=write(serial_fd, data, size);
	if(written<size){
		perror("Error: serial write incomplete:");
		return STATUS_ERROR;	//Couldn't send data completely
	}
	return STATUS_OK; //All went well, nothing to return
}


uint32_t SerialCommunication::read_descriptor() {
	uint8_t read_descriptor[7]={0};
	ssize_t read_size = read(serial_fd, read_descriptor, 7);
//	for(uint32_t i=0;i<read_size;i++){
//		printf("%#02x ", read_descriptor[i]);
//	}
//	printf("\n");
	if(read_size<7){
		printf("Error: serial descriptor read incomplete: read %d/7 bytes\n", (uint32_t)read_size);
		return 0; //Return 0, let the user handle what happens
	}
	if(read_descriptor[0]!=START_FLAG || read_descriptor[1]!=START_FLAG_2){
		printf("Error: serial descriptor start flags wrong, got %#02x:%#02x\n", read_descriptor[0], read_descriptor[1]);
		return 0;
	}
	uint32_t response_data_len=0;
	for(int i=2;i<5;i++){
		response_data_len|=read_descriptor[i]<<(i-2);
	}
	response_data_len|=read_descriptor[5]&0x3F; // size is 30 bits, not 32: last byte is reduce with a mast 0x00111111
	return response_data_len;
}

/**
 * Reads n bytes of data, assumes the reader later calls delete[] on the pointer returned
 * @param num_bytes
 * @return pointer to an array of num_bytes of read_data, in little endian format
 */
uint8_t *SerialCommunication::read_data(uint32_t num_bytes) {
	uint8_t* read_data=new uint8_t[num_bytes];
	ssize_t read_size = read(serial_fd, read_data, num_bytes);
	if(read_size<num_bytes){
		printf("Error: read less data bytes than expected, got %d/%d\n", (uint32_t)read_size, num_bytes);
		return nullptr;
	}
	return read_data;
}

uint8_t SerialCommunication::read_byte() {
	uint8_t read_data;
	ssize_t read_size = read(serial_fd, &read_data, 1);
	if(read_size<1){
		printf("Error: read 0 bytes\n");
		return 0;
	}
	return read_data;
}
