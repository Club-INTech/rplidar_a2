//
// Created by tic-tac on 10/1/18.
//

#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "SerialCommunication.hpp"

SerialCommunication::SerialCommunication(const char *filePath, uint32_t baudrate, int parity) {
	serial_fd=open(filePath, O_RDWR);
	set_interface_attribs(B115200, 0); 					//8N1 at 115200
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
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

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
	tty.c_cc[VTIME] = 5;            	// 0.5 seconds read timeout

	if (tcsetattr (serial_fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes", errno);
}

/**
 * Sends data from a RequestPacket reference
 * @param packet
 */
void SerialCommunication::send_packet(const RequestPacket& packet) {
	memset(data, 0, rp_values::MAX_PAYLOAD);
	uint8_t size=packet.get_packet(data);
	ssize_t written=write(serial_fd, data, size);
	if(written<size){
		perror("Error: serial write incomplete:");
		exit(EXIT_FAILURE);
	}
	if(!(packet.order == rp_values::OrderByte::STOP ||
		  packet.order == rp_values::OrderByte::RESET ||
		  packet.order == rp_values::OrderByte::SET_PWM))
	{
		ssize_t read_size = read(serial_fd, data, 10);
		if (read_size > 0) {
			printf("READ %d bytes\n", (int) read_size);
			for (int i = 0; i < read_size; i++) {
				printf("%#02x ", data[i]);
			}
			printf("\n");
		}
	}
}

