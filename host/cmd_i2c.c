
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

#include "cmd.h"

#define I2C_ADDR 0x32

// first byte is junk (?)
// second byte is command
int i2c_read(int f, uint8_t *buf, int len) {
	if (read(f, buf, len) != len)
	{
		printf("Failed to read from the i2c bus.\n");
		return 0;
	}
	else
	{
		#ifdef DEBUG
		for(int i=0; i< len;i++)
			printf("READ: %x\n", buf[i]);
		#endif
		return 1;
	}
}

int i2c_write(int f, uint8_t *buf, int len) {
	if (write(f, buf, len) != len)	
	{
		printf("Failed to write to the i2c bus.\n");
		return 0;
	} else {
		#ifdef DEBUG
		for(int i=0; i< len;i++)
			printf("WRITE: %x\n", buf[i]);
		#endif

		return 1;
	}
}

int i2c_xfer(int f, uint8_t *data_in, uint8_t *data_out, int len) {
	if (i2c_write(f, data_in, len)) {
		return i2c_read(f, data_out, len+1);
	} else {
		return 0;
	}
}

int test(int f) {
	uint8_t in[2];
	uint8_t out[3];
	in[0] = CMD_PING;
	in[1] = 0xAA;
	printf("lifepo4 PSU testing ... ");
	i2c_xfer(f, in, out, 2);
	if (out[2] == 0xAA) {
		printf("OK\n");
		return 1;
	}
	else {
		printf("KO\n");
		return 0;
	}
}

int dump(int f) {
	if (test(f)) 
	{
		uint8_t in[4];
		uint8_t out[5];
		in[0] = CMD_STATE;

		printf("lifepo4 PSU state ... ");
		i2c_xfer(f, in, out, 2);
		printf("STATE: %x\n", out[2]);

		in[0] = CMD_BAT_V;

		printf("lifepo4 PSU BAT V ... ");
		i2c_xfer(f, in, out, 4);
		printf("BAT V: %0.2f\n", (out[2] + out[3] / 10.0 + out[4] / 100.0));

		in[0] = CMD_BAT_A;

		printf("lifepo4 PSU BAT A ... ");
		i2c_xfer(f, in, out, 4);
		printf("BAT A: %0.2f\n", (out[2] + out[3] / 10.0 + out[4] / 100.0));

		in[0] = CMD_INP_A;

		printf("lifepo4 PSU INPUT A ... ");
		i2c_xfer(f, in, out, 4);
		printf("INP A: %0.2f\n", (out[2] + out[3] / 10.0 + out[4] / 100.0));
	}

}

int main(int argc, char *argv[]) {

	int file_i2c;
	int length;
	unsigned char in[16] = {0};
	unsigned char out[16] = {0};


	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 1;
	}

	if (ioctl(file_i2c, I2C_SLAVE, I2C_ADDR) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 1;
	}


	dump(file_i2c);

	return 0;
}
