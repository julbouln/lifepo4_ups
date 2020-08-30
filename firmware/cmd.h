#ifndef CMD_H
#define CMD_H

#include <stdint.h>

// test
#define CMD_PING 0x10 

// read
#define CMD_STATE 0x20
#define CMD_BAT_V 0x30
#define CMD_BAT_A 0x32
#define CMD_INP_A 0x34
#define CMD_BAT_G 0x36

// write
#define CMD_CTL 0x40

#define USB_CMD 1
#define I2C_CMD 2

void cmd_parse(char *recv, uint8_t recv_len, uint8_t *send, uint8_t *send_len);

#endif