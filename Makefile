# Makefile for irkeylog
#
# Instructions
#
# To make the .hex file:
# make all
#
# To burn the .hex file:
# make isp

# program name should not be changed...
PROGRAM    = irkeylog

# enter the parameters for the avrdude isp tool
ISPTOOL	    = arduino
ISPPORT	    = /dev/ttyUSB0
ISPSPEED    = -b 115200

MCU_TARGET  = atmega328p
DEVICE_PART = m328p
LDSECTION   = --section-start=.text=0x0

# the lock bits should be 0xff instead of 0x3f, but since the high two bits
# of the lock byte are unused, avrdude would get confused.

ISPFUSES    = avrdude -c $(ISPTOOL) -p $(DEVICE_PART) -P $(ISPPORT) $(ISPSPEED) \
-e -u -U lock:w:0x3f:m -U efuse:w:0x$(EFUSE):m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m
ISPFLASH    = avrdude -c $(ISPTOOL) -p $(DEVICE_PART) -P $(ISPPORT) $(ISPSPEED) \
-U flash:w:$(PROGRAM).hex -U lock:w:0x3f:m

OBJ        = $(PROGRAM).o
OPTIMIZE   = -O2 -Os

DEFS       =
LIBS       =

CC         = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -fshort-enums -DF_CPU=$(AVR_FREQ) $(DEFS)
override LDFLAGS       = -Wl,$(LDSECTION)
#override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)

CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>4'
AVR_FREQ = 16000000L 

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PROGRAM).hex

isp: HFUSE = DF
isp: LFUSE = D2
isp: EFUSE = FF
isp: all
	# $(ISPFUSES)
	$(ISPFLASH)

%.elf: %.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
