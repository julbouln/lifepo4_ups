all:
	gcc -o lifepo4_cmd_usb cmd_usb.c -lm -lusb-1.0
	gcc -o lifepo4_cmd_i2c cmd_i2c.c -lm
	gcc -o lifepo4_cmd_uart cmd_uart.c -lm
clean:
	rm lifepo4_cmd_usb lifepo4_cmd_i2c lifepo4_cmd_uart idle