NAME   = demoh103_blink_rom

OBJDIR	= .
TARGET	= main

CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump
OOCD	= openocd-stm32
BL	= ../../tools/stm32loader.py
  
CFLAGS  =  -I./ -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -Tstm_h103_blink_rom.cmd -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

OOCD_ADAPTER	= arm-usb-ocd
OOCD_TARGET	= olimex_stm32_h103


all: test

clean:
	-rm -f main.lst crt.o main.o main.out main.hex main.map stm32f10x_rcc.o stm32f10x_gpio.o

upload-serial:
	@ echo "...uploading"
	$(BL) -e -w -v -p /dev/ttyUSB0 main.bin

upload:  all
	@echo -e "  OOCD\t$<"
	$(Q)$(OOCD) -f stm32.cfg \
                -c init \
                -c "reset halt" \
                -c "flash write_image erase main.bin 0x08000000" \
                -c reset \
                -c shutdown

test: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.list

main.out: main.o stm32f10x_rcc.o stm32f10x_gpio.o stm_h103_blink_rom.cmd 
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.out main.o stm32f10x_rcc.o stm32f10x_gpio.o

stm32f10x_rcc.o: lib/stm32f10x_rcc.c 
	@ echo ".compiling"
	 $(CC) $(CFLAGS) lib/stm32f10x_rcc.c 
	 
stm32f10x_gpio.o: lib/stm32f10x_gpio.c
	@ echo ".compiling"
	 $(CC) $(CFLAGS) lib/stm32f10x_gpio.c 
	 
main.o: main.c
	@ echo ".compiling"
	 $(CC) $(CFLAGS) main.c
