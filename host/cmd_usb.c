#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <libusb-1.0/libusb.h>

#include "cmd.h"

#define PACKET_SIZE 64
#define VENDORID 0xF055
#define PRODUCTID 0xCAFE

//#define DEBUG

static struct libusb_device_handle *devh = NULL;

static int ep_in_addr  = 0x82;
static int ep_out_addr = 0x01;

#define IN_BUFFER_LEN 1024*8
static uint8_t in_buffer[IN_BUFFER_LEN];
int recv_bytes = 0;
int received = 0;

int trfr_bytes = 0;
int transfered = 0;

// OUT-going transfers (OUT from host PC to USB-device)
struct libusb_transfer *transfer_out = NULL;

// IN-coming transfers (IN to host PC from USB-device)
struct libusb_transfer *transfer_in = NULL;


static libusb_context *ctx = NULL;



static int _usb_needed_len(int len) {
//  len + (len >> 6) + 1;
//  return len + (len / (IFUSB_PACKET_SIZE - 1)) + 1;
	return (int)ceil(((float)len / ((float)(PACKET_SIZE - 1))) * (float)PACKET_SIZE);
}


void usb_packet_write(uint8_t *buf, uint8_t len)
{
	int r;
	int actual_length;
	if ((r = libusb_bulk_transfer(devh, ep_out_addr, buf, len,
	                              &actual_length, 10)) < 0) {
		fprintf(stderr, "usb_packet_write blocking transfer error: %d\n", r);
	}
}

int usb_packet_read(uint8_t *buf, uint8_t len)
{
	int actual_length;
	int rc = libusb_bulk_transfer(devh, ep_in_addr, buf, len, &actual_length,
	                              10);
	if (rc == LIBUSB_ERROR_TIMEOUT) {
//    printf("ifusb timeout (%d)\n", actual_length);
		return -1;
	} else if (rc < 0) {
		fprintf(stderr, "usb_packet_read blocking transfer error: %d\n", rc);
		return -1;
	}

	return actual_length;
}


void usb_xfer(uint8_t id, uint8_t *data_in, uint8_t *data_out, int len) {
	int total_len = _usb_needed_len(len);
	uint8_t tmpbuf[total_len];
	int cur = 0;
	int sent_packets = 0;
	int sent_bytes;

	for (sent_bytes = 0; sent_bytes < total_len; sent_bytes += (PACKET_SIZE - 1)) {
		int i;
		uint8_t t_data_in[PACKET_SIZE];
		uint8_t t_data_out[PACKET_SIZE];
		uint8_t send = PACKET_SIZE;
		int remaining = total_len - sent_bytes - sent_packets;
		if (remaining < PACKET_SIZE)
			send = remaining;

		if (send == 0)
			break;

		// add command id
		t_data_in[0] = id;
		tmpbuf[cur] = id;
		cur++;
		for (i = 1; i < send; i++) {
			t_data_in[i] = data_in[sent_bytes + i - 1];
			tmpbuf[cur] = data_in[sent_bytes + i - 1];
			cur++;
		}

		usb_packet_write(t_data_in, send);

		// merge received packets
		usb_packet_read(t_data_out, send);
		for (i = 1; i < send; i++) {
			data_out[sent_bytes + i - 1] = t_data_out[i];
#ifdef DEBUG
		printf("usb xfer %x\n", t_data_out[i]);
#endif

		}
		sent_packets++;
	}
}

int test() {
	uint8_t buf[1];
	uint8_t recv[1];
	buf[0] = 0xAA;
	printf("lifepo4 PSU testing ... ");
	usb_xfer(CMD_PING, buf, recv, 1);
	if (recv[0] == 0xAA) {
		printf("OK\n");
		return 1;
	}
	else
		return 0;
}

int main(int argc, char *argv[]) {
	int rc;

	/* Initialize libusb
	 */
	rc = libusb_init(NULL);
	if (rc < 0) {
		fprintf(stderr, "lifepo4 PSU error initializing libusb: %s\n", libusb_error_name(rc));
		return 0;
	}

	/* Set debugging output to max level.
	 */
	libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_WARNING);

	/* Look for a specific device and open it.
	 */
	devh = libusb_open_device_with_vid_pid(ctx, VENDORID, PRODUCTID);
	if (!devh) {
		fprintf(stderr, "lifepo4 PSU error finding USB device %x:%x\n", VENDORID, PRODUCTID);
		return 0;
	}

	/*
	* Class defines two interfaces: the Control interface and the
	* Data interface.
	*/
	int if_num;
	for (if_num = 0; if_num < 1; if_num++) {
		rc = libusb_claim_interface(devh, if_num);
		if (rc < 0) {
			fprintf(stderr, "lifepo4 PSU error claiming interface %d: %s\n", if_num,
			        libusb_error_name(rc));
			return 0;
		}
	}

	sleep(1);
	if (!test()) {
		fprintf(stderr, "lifepo4 PSU error testing protocol\n");
		return 0;
	}


	uint8_t buf[4];
	uint8_t recv[4];
	buf[0] = 0x00;
	recv[0] = 0x00;
	printf("lifepo4 PSU state ... ");
	usb_xfer(CMD_STATE, buf, recv, 1);
	printf("STATE: %x\n",recv[0]);

	buf[0] = 0x00;
	recv[0] = 0x00;

	printf("lifepo4 PSU BAT V ... ");
	usb_xfer(CMD_BAT_V, buf, recv, 3);
	printf("BAT V: %0.2f\n",(recv[0]+recv[1]/10.0+recv[2]/100.0));

	buf[0] = 0x00;
	recv[0] = 0x00;

	printf("lifepo4 PSU BAT A ... ");
	usb_xfer(CMD_BAT_A, buf, recv, 3);
	printf("BAT A: %0.2f\n",(recv[0]+recv[1]/10.0+recv[2]/100.0));

	buf[0] = 0x00;
	recv[0] = 0x00;

	printf("lifepo4 PSU INPUT A ... ");
	usb_xfer(CMD_INP_A, buf, recv, 3);
	printf("INP A: %0.2f\n",(recv[0]+recv[1]/10.0+recv[2]/100.0));

}