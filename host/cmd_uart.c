#define TERMINAL    "/dev/ttyUSB0"

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "cmd.h"

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}



int uart_xfer(int fd, uint8_t *data_in, uint8_t *data_out, int in_len, int out_len) {
	int wlen, rdlen;
    wlen = write(fd, data_in, in_len);
/*	for(int i = 0; i<wlen;i++) {
		printf("W%d:%x ",i,data_in[i]);
	}*/
    if (wlen != in_len) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output */

	rdlen = read(fd, data_out, out_len);
/*	for(int i = 0; i<rdlen;i++) {
		printf("R%d:%x ",i,data_out[i]);
	}*/
	printf("\n");

}

int test(int f) {
	uint8_t in[2];
	uint8_t out[2];
	in[0] = CMD_PING;
	in[1] = 0xAA;
	printf("lifepo4 PSU testing ... ");
	uart_xfer(f, in, out, 2, 2);
	if (out[1] == 0xAA) {
		printf("OK\n");
		return 1;
	}
	else {
		printf("KO\n");
		return 0;
	}
}

int dump(int fd) {
	if(test(fd)) {
		uint8_t in[4];
		uint8_t out[4];
		in[0] = CMD_STATE;

		printf("lifepo4 PSU state ... ");
		uart_xfer(fd, in, out, 1, 2);
		printf("STATE: %x\n", out[1]);

		in[0] = CMD_BAT_V;

		printf("lifepo4 PSU BAT V ... ");
		uart_xfer(fd, in, out, 1, 4);
		printf("BAT V: %0.2f\n", (out[1] + out[2] / 10.0 + out[3] / 100.0));

		in[0] = CMD_BAT_A;

		printf("lifepo4 PSU BAT A ... ");
		uart_xfer(fd, in, out, 1, 4);
		printf("BAT A: %0.2f\n", (out[1] + out[2] / 10.0 + out[3] / 100.0));

		in[0] = CMD_INP_A;

		printf("lifepo4 PSU INPUT A ... ");
		uart_xfer(fd, in, out, 1, 4);
		printf("INP A: %0.2f\n", (out[1] + out[2] / 10.0 + out[3] / 100.0));

		in[0] = CMD_BAT_G;

		printf("lifepo4 PSU BAT G ... ");
		uart_xfer(fd, in, out, 1, 2);
		printf("BAT G: %d\n", out[1]);

	}
}

int main(int argc, char *argv[])
{
    char *portname = TERMINAL;
    int fd;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */


    if(argc > 1) {
        if (strcmp(argv[1],"dump") == 0) {
            dump(fd);
        } else {
            if(strcmp(argv[1], "control") == 0 && argc > 2) {
                uint8_t ctrl = atoi(argv[2]);
                uint8_t in[2];
                uint8_t out[2];
                in[0] = CMD_CTL;
                in[1] = ctrl;
                printf("lifepo4 PSU control ... ");
                uart_xfer(fd, in, out, 2, 2);
                printf("%x\n",out[1]);
            }
        }
    }
}