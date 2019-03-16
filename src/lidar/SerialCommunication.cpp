//
// Created by tic-tac on 10/1/18.
//
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "lidar/SerialCommunication.hpp"

#include "lidar/SerialCommunication.hpp"

using namespace rp_values;

void SerialCommunication::init(const char *filePath, speed_t baudrate, int parity) {
	serial_fd=open(filePath, O_RDWR);		//Get a file descriptor to the serial device, usually /dev/ttyUSB0
	if(serial_fd==-1){
		printf("Error: Serial device not found at %s\n", filePath);
		exit(EXIT_FAILURE);
	}
	path=std::string(filePath);
	set_interface_attribs(baudrate, parity); 	//sets up file attributes with termios for serial communication (8N1)
	set_blocking(true, 5);						//Blocking communication, with timeout
	setDTR(false);								//DTR disables PWM control of the motor, so it should be disabled
	memset(data, 0, rp_values::REQUEST_SIZE);	//Initialize output data buffer to 0
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
 * Sets the attributes of a file for Serial communication
 * @param speed : the baudrate (of type speed_t), usually B115200
 * @param parity : parity of the communication(usually 0)
 * @return
 */
int SerialCommunication::set_interface_attribs(speed_t speed, int parity) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (serial_fd, &tty) < 0)
	{
		perror ("error from tcgetattr");
		return -1;
	}

	/*
	 Input flags - Turn off input processing

	 convert break to null byte, no CR to NL translation,
	 no NL to CR translation, don't mark parity errors or breaks
	 no input parity check, don't strip high bit off,
	 no XON/XOFF software flow control */
	tty.c_iflag &=~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);


	/*
	 Output flags - Turn off output processing

	 no CR to NL translation, no NL to CR-NL translation,
	 no NL to CR translation, no column 0 CR suppression,
	 no Ctrl-D suppression, no fill characters, no case mapping,
	 no local output processing

	 config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST); */
	tty.c_oflag = 0;

	/*
	 No line processing

	 echo off, echo newline off, canonical mode off,
	 extended input processing off, signal chars off */
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/*
	 Turn off character processing

	 clear current char size mask, no parity checking,
	 no output processing, force 8 bit input */
	tty.c_cflag &= ~(CSIZE | (parity?PARENB:0));
	tty.c_cflag |= CS8;

	/*
	 One input byte is enough to return from read()
	 Inter-character timer off */

	tty.c_cc[VMIN]  = 1;
	tty.c_cc[VTIME] = 0;

	/*
	 Communication speed (simple version, using the predefined
	 constants) */
	if(cfsetispeed(&tty, speed) < 0 || cfsetospeed(&tty, speed) < 0) {
		printf("Error: cfsetispeed | cfsetospeed\n");
		exit(EXIT_FAILURE);
	}

//	cfsetospeed (&tty, speed);
//	cfsetispeed (&tty, speed);
//
//	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
//	// disable IGNBRK for mismatched speed tests; otherwise receive break
//	// as \000 chars
//		tty.c_iflag &= ~IGNBRK;         // ignore break signal
//	tty.c_lflag = 0;                // no signaling chars, no echo,
//	// no canonical processing
//	tty.c_oflag = 0;                // no remapping, no delays
//	tty.c_cc[VMIN]  = 0;            // read doesn't block
//	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
//
//	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
//
//	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
//	// enable reading
//	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
//
//	tty.c_cflag |= parity;
//	tty.c_cflag &= ~CSTOPB;
//	tty.c_cflag &= ~CRTSCTS;

	//Saves the attributes and applies them to the tty on 'serial_fd'
	if (tcsetattr (serial_fd, TCSAFLUSH, &tty) < 0)
	{
		printf ("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}


/**
 * Sets blocking state of the file descriptor interface
 * @param should_block or not
 * @param timeout in tenths of seconds
 */
void SerialCommunication::set_blocking(bool should_block, uint8_t timeout) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (serial_fd, &tty) != 0)
	{
		printf ("error %d from tggetattr\n", errno);
		return;
	}
	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = timeout;            	// 0.1*timeout seconds read timeout
	if (tcsetattr (serial_fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes\n", errno);
}


/**
 * Sends data from a RequestPacket reference
 * @param packet
 */
ComResult SerialCommunication::send_packet(const RequestPacket &packet) {
	memset(data, 0, REQUEST_SIZE);
	uint8_t size=packet.get_packet(data);			//Fills data[REQUEST_SIZE] with the bytes of the request packet, and gets the number of bytes to write
	ssize_t written=write(serial_fd, data, size);	//Tries to write those bytes to serial buffer on serial_fd, and get the amount of bytes written
	if(written==-1){
		printf("Error: No Connection to LiDAR on device %s\nQuitting\n", path.c_str());
		exit(EXIT_FAILURE);
	}
	if(written<size){
		perror("Error: serial write incomplete:\n");
		return STATUS_ERROR;	//Couldn't send data completely
	}
	return STATUS_OK; //All went well, nothing to return
}


/**
 * Reads a response descriptor incoming from LiDAR, and selects the size of the next data packets if there are any
 * @return size of the next incoming data packets (for SCAN, FORCE_SCAN, EXPRESS_SCAN orders)
 */
uint32_t SerialCommunication::read_descriptor() {
	uint8_t read_descriptor[rp_values::DESCRIPTOR_SIZE]={0};
	ssize_t read_size = read(serial_fd, read_descriptor, 7);	//Reads from the serial_fd buffer, copies it to read_descriptor[7]
//	for(int i=0;i<rp_values::DESCRIPTOR_SIZE;i++){
//		printf("0x%02x ", read_descriptor[i]);
//	}
//	printf("\n");
	if(read_size<rp_values::DESCRIPTOR_SIZE){
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
	response_data_len|=read_descriptor[5]&0x3F; // size is 30 bits, not 32: last byte is reduced with a mask 0x00111111
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

/**
 * Reads one byte of data
 * @return byte read from serial_fd buffer
 */
uint8_t SerialCommunication::read_byte() {
	uint8_t read_data;
	ssize_t read_size = read(serial_fd, &read_data, 1);
	if(read_size<1){
		printf("Error: read 0 bytes\n");
		return 0;
	}
	return read_data;
}


/**
 * Reads all available data in the RX buffer
 */
void SerialCommunication::flush() {
	char buff[1];
	ssize_t res=1;
	while(res>0){
		set_blocking(false,0);
		res=read(serial_fd, buff, 1);
		set_blocking(true, 5);
	}
}
