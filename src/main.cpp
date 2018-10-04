#include "RequestPacket.hpp"
#include "tcp.h"
#include "LinkedValuesList.h"
#define NB_CABINS (16)
bool on = true;

unsigned long time() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec*(unsigned long)1000000+tv.tv_usec;
}

void setDTR(int desc, int dtrEnable)
{
	int flags;
	ioctl(desc, TIOCMGET, &flags);
	dtrEnable!=0 ? (flags |= TIOCM_DTR) : (flags &= ~TIOCM_DTR);
	ioctl(desc, TIOCMSET, &flags);
}

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error from tcgetattr", errno);
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

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            	// 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes", errno);
}


void initLidar(int fileDesc)
{
	uint8_t * data = (uint8_t*) malloc(sizeof(uint8_t)*10);

	RequestPacket packet=RequestPacket(OrderByte::GET_HEALTH);
	packet.get_packet(data);

	write(fileDesc, data, sizeof(uint8_t)*packet.payload_size+2);
	read(fileDesc, data, sizeof(uint8_t)*10);
	data[0] = (uint8_t) 0xA5;
	data[1] = (uint8_t) 0xF0;
	data[2] = (uint8_t) 0x02;
	data[3] = (uint8_t) 0x94;
	data[4] = (uint8_t) 0x02;
	data[5] = (uint8_t) (0 ^ 0xA5 ^ 0xF0 ^ 2 ^ 0x94 ^ 0x02);

	write(fileDesc, data, sizeof(uint8_t)*6);

	data[0] = (uint8_t) 0xA5;
	data[1] = (uint8_t) 0x82;
	data[2] = (uint8_t) 0x05;
	data[3] = (uint8_t) 0x00;
	data[4] = (uint8_t) 0x00;
	data[5] = (uint8_t) 0x00;
	data[6] = (uint8_t) 0x00;
	data[7] = (uint8_t) 0x00;
	data[8] = (uint8_t) 0x22;

	write(fileDesc, data, sizeof(uint8_t)*9);
	read(fileDesc, data, sizeof(uint8_t)*7);

	free(data);
}


void stopLidar(int fileDesc)
{
	uint8_t * data = (uint8_t*) malloc(sizeof(uint8_t)*4);

	data[0] = (uint8_t) 0xA5;
	data[1] = (uint8_t) 0x25;
	data[2] = (uint8_t) 0x00;
	data[3] = (uint8_t) (0^data[0]^data[1]^data[2]);
	write(fileDesc, data, sizeof(uint8_t)*4);
	printf("Sent stop order\n");
	printf("%d %d %d %d\n", data[0], data[1], data[2], data[3]);

	write(fileDesc, data, sizeof(uint8_t)*2);

	free(data);

	data=(uint8_t*) malloc(sizeof(uint8_t)*6);
	data[0] = (uint8_t) 0xA5;
	data[1] = (uint8_t) 0xF0;
	data[2] = (uint8_t) 0x02;
	data[3] = (uint8_t) 0x00;
	data[4] = (uint8_t) 0x00;
	data[5] = (uint8_t) (0^data[0]^data[1]^data[2]^data[3]^data[4]);
	write(fileDesc, data, sizeof(uint8_t)*6);
	printf("Sent stop order\n");
	printf("%d %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4], data[5]);

	free(data);
}


//returns checksum
uint8_t findPacketStart(int fileDesc, FILE* outDesc, char out)
{
	char found = 0;
	char skip = 0;
	uint8_t checksum = 0;
	uint8_t data = 0;

	while (!found)
	{

		if(!skip)
		{
			read(fileDesc, &data, 1);
			if(out) fwrite(&data, 1, 1, outDesc);
		}
		else skip = 0;


		if((data & (uint8_t)0xF0) == 0xA0)
		{

			checksum = data & 0xF;

			read(fileDesc, &data, 1);

			if(out) fwrite(&data, 1, 1, outDesc);

			if((data & (uint8_t)0xF0) == 0x50)
			{

				checksum |= ((data & (uint8_t)0xF) << 4);
				found = 1;
			}
			else
			{
				skip = 1;
			}
		}
	}

	return checksum;
}



int resetScan(uint8_t byte)
{
	return byte & (uint8_t)0x80;
}


float* paquetExtractor(uint16_t startAngle_q8, uint16_t angleDiff_q8, uint8_t* data)
{
	float* res = (float*) malloc(sizeof(float)*NB_CABINS*4);

	uint16_t dist1_q2;
	uint16_t dist2_q2;
	uint8_t dAngle1_q3;
	uint8_t dAngle2_q3;

	for(char i = 0 ; i < NB_CABINS ; i++)
	{
		dist1_q2 = ((uint16_t)(data[(i*5)] & (uint8_t)0xFC) >> 2) | ((uint16_t)data[(i*5)+1] << 6);
		dAngle1_q3 = ((data[(i*5)] & (uint8_t)0x03) << 4) | (data[(i*5)+4] & (uint8_t)0x0F);
		dist2_q2 = ((uint16_t)(data[(i*5)+2] & (uint8_t)0xFC) >> 2) | ((uint16_t)data[(i*5)+3] << 6);
		dAngle2_q3 = ((data[(i*5)+2] & (uint8_t)0x03) << 4) | ((data[(i*5)+4] & (uint8_t)0xF0) >> 4);



		res[(i*4)] = (float)(dist1_q2);

		res[(i*4)+1] = (float)(startAngle_q8 + (angleDiff_q8 * (i*2)) )    / (1 << 8);
		//  - ((dAngle1_q3 & (uint8_t)0x1F)) << 5) / (1 << 8) * ((dAngle1_q3 & (uint8_t)0x20) == 0 ? 1.f : -1.f);

		res[(i*4)+2] = (float)(dist2_q2);

		res[(i*4)+3] = (float)(startAngle_q8 + (angleDiff_q8* ((i*2)+1))  ) / (1 << 8) ;
		// - ((dAngle2_q3 & (uint8_t)0x1F)) << 5) / (1 << 8) * ((dAngle2_q3 & (uint8_t)0x20) == 0 ? 1.f : -1.f);
	}


	return res;
}


int16_t q_div(int16_t a, int16_t b, int Q)
{
	/* pre-multiply by the base (Upscale to Q16 so that the result will be in Q8 format) */
	int32_t temp = (int32_t)a << Q;
	/* Rounding: mid values are rounded up (down for negative values). */
	/* OR compare most significant bits i.e. if (((temp >> 31) & 1) == ((b >> 15) & 1)) */
	if((temp >= 0 && b >= 0) || (temp < 0 && b < 0)) {
		temp += b / 2;    /* OR shift 1 bit i.e. temp += (b >> 1); */
	} else {
		temp -= b / 2;    /* OR shift 1 bit i.e. temp -= (b >> 1); */
	}
	return (int16_t)(temp / b);
}

void sig_handler(int signo)
{
	if (signo == SIGINT)
		on = 0;
}

int main(int argc, char** argv) {
	if(argc == 4 && (strcmp(argv[1], "-o")))
	{
		printf("BAD");
		printf("usage : ./RPLidarINTechLib [-o <raw output file>] <serial port>");
		return -1;
	}
	else if (argc != 2 && argc != 4) {
		printf("BAD");
		printf("usage : ./RPLidarINTechLib [-o <raw output file>] <serial port>");
		return -1;
	}

	char out = 0;

	if(argc == 4) out = 1;

	signal(SIGINT, sig_handler);
	signal(SIGPIPE, SIG_IGN); // O

	int serialDesc = open(argv[argc-1], O_RDWR);

	FILE* outDesc = NULL;
	if(out)outDesc = fopen(argv[2], "w");

	if (serialDesc < 0) {
		printf("error %d opening %s: %s", errno, argv[argc-1], strerror(errno));
		return -1;
	}

	set_interface_attribs(serialDesc, B115200, 0);        // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking(serialDesc, 1);
	setDTR(serialDesc, 0);

	int sock = 0;
	int c = 0;

	while (on && c != 10000)
	{
		unsigned long t = time();

		uint8_t *data = (uint8_t *) malloc(sizeof(uint8_t) * 84);
		uint8_t *lastData = (uint8_t *) malloc(sizeof(uint8_t) * 84);
		uint8_t *temp;

		char init = 0;
		uint8_t checksum;
		uint8_t recvChecksum;

		uint16_t next_angle_q6;
		uint16_t angle_q6;
		float next_angle;
		float angle;

		LinkedValuesList *turn = NULL;
		float lastAngle = -1000.f;

		sock = server_wait();

		int up = 1;

		initLidar(serialDesc);

		while (up && c != 10000) {
			checksum = findPacketStart(serialDesc, outDesc, out);

			read(serialDesc, data, 78);

			if(out) fwrite(&data, 1, 78, outDesc);

			recvChecksum = 0x00;

			for(int b = 0 ; b < 78 ; b++)
			{
				recvChecksum ^= data[b];
			}

			//if(checksum != recvChecksum)
			//{
			//    printf("BAD DATA RECV\n");
			//    init = 0;
			//    continue;
			//}

			if (resetScan(data[1])) {
				init = 0;
				free(turn);
				turn = NULL;
				lastAngle = -1000.f;
			}

			if (init) {
				next_angle_q6 = q_div((uint16_t) data[0] | ((uint16_t) (data[1] & (uint8_t) 0x7F) << 8), 64 ,6);
				angle_q6 = q_div((uint16_t) lastData[0] | ((uint16_t) (lastData[1] & (uint8_t) 0x7F) << 8), 64, 6);

				next_angle = (((float) next_angle_q6) * 0.015625f);
				angle = (((float) angle_q6) * 0.015625f);

				float *values = paquetExtractor(angle_q6 << 2,
				                                ((next_angle < angle ? 360 << 8 : 0) + (next_angle_q6 << 2) -
				                                 (angle_q6 << 2)) / (32 << 8), lastData + 2);

				for (int i = 0; i < NB_CABINS * 2; i++) {
					if (values[(i * 2) + 1] >= 360.f || values[(i * 2) + 1] < lastAngle) {
						up = send_turn(sock, turn) < 0 ? 0 : 1;

						turn = NULL;
						lastAngle = -1000.f;
					}

					LinkedValuesList *val = (LinkedValuesList *) malloc(sizeof(LinkedValuesList));
					val->next = turn;
					val->dist = values[(i * 2)];
					val->angle = values[(i * 2) + 1] - (values[(i * 2) + 1] >= 360.f ? 360.f : 0.f);
					turn = val;
				}

				lastAngle = values[NB_CABINS * 4 - 1];

			} else init = 1;

			temp = lastData;
			lastData = data;
			data = temp;

			if (!(++c % 100)) {
				printf("%lu\n", time() - t);
				t = time();
			}

		}

		stopLidar(serialDesc);

		free(data);
		free(lastData);
		free(turn);

		close(sock);
		close_server();
	}

	close(serialDesc);
	fclose(outDesc);
	close(sock);
	close_server();

	return 0;
	return 0;
}