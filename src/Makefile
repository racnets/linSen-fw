include ../Makefile.common

#select only one of these
#OBJS += blinkyIRQ_example.o
#OBJS += blinky_example.o
#OBJS += USART_example.o
OBJS += main.o led.o usart1.o tsl1401.o blockmatch.o button.o mavlink_bridge.o calibration.o time.o i2c.o eeprom.o quadpixemd.o filter.o

all: src

src: src.a

src.a: $(OBJS)
	$(AR) cr src.a $(OBJS)

.PHONY: src clean
clean:
	rm -f *.a *.o

