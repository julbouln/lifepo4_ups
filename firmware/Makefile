DEVICE          = stm32f042f6
#DEVICE          = stm32f070f6
OPENCM3_DIR     = ../libopencm3
OBJS            += rtc.o pwm.o i2c.o usart.o usb.o power_manager.o gauge.o cmd.o

CFLAGS          += -Os -ggdb3
CPPFLAGS		+= -MD
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: ups.elf ups.hex

clean:
	$(Q)$(RM) -rf ups.* *.o *.d generated*.ld

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk

prog: ups.bin
	sudo dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D ups.bin

prog_uart: ups.bin
	stm32flash -e 255 -v -w ups.bin /dev/ttyUSB0
