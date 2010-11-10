TARGET = main
CONFIG = ''
BUILDDIR = build
OBJDIR = build/obj
BINDIR = build/bin
MAVLINKBASEDIR = ../mavlink/include
MAVLINKDIR = ../mavlink/include/pixhawk
MAVLINKUSERDIR = ../mavlink/include/user
Q = @
TOOLS = tools

SRCARM += main/mainloop_generic.c
SRCARM += main/mainloop_fixed_wing.c
SRCARM += main/mainloop_quadrotor.c
SRCARM += main/mainloop_ground_car.c
SRCARM += main/common_mainloop_functions.c
SRCARM += math/lookup_sin_cos.c
SRCARM += arm7/adc.c
SRCARM += arm7/armVIC.c 
SRCARM += arm7/cam_trigger.c 
SRCARM += arm7/dac.c 
SRCARM += arm7/led.c  
SRCARM += arm7/spi.c
SRCARM += arm7/i2c.c
SRCARM += arm7/spi_devices/ads8341.c
SRCARM += arm7/spi_devices/ms2100.c
SRCARM += arm7/spi_devices/sca3100.c
SRCARM += arm7/spi_devices/sca3000.c
SRCARM += arm7/i2c_devices/bmp085.c
SRCARM += arm7/i2c_devices/eeprom.c
SRCARM += arm7/i2c_devices/i2c_motor_controller.c
SRCARM += arm7/i2c_devices/i2c_motor_mikrokopter.c
SRCARM += arm7/i2c_devices/hmc5843.c
SRCARM += arm7/ppm.c 
SRCARM += arm7/pwm.c 
SRCARM += arm7/sys_time.c 
SRCARM += arm7/uart.c 
SRCARM += comm/comm.c
SRCARM += system/watchdog.c
SRCARM += system/pid.c
SRCARM += hal/shutter.c
SRCARM += system/communication.c
SRCARM += controllers/coaxial/control_position.c
SRCARM += controllers/coaxial/control_attitude.c
SRCARM += controllers/coaxial/control_yawSpeed.c
SRCARM += controllers/coaxial/control_yaw_altitude.c
SRCARM += controllers/quadrotor/control_quadrotor_attitude.c
SRCARM += controllers/fixed_wing/control_fixed_wing_attitude.c
SRCARM += controllers/quadrotor/control_quadrotor_position.c
SRCARM += controllers/quadrotor/control_quadrotor_start_land.c
SRCARM += system/remote_control.c
SRCARM += system/debug.c
SRCARM += system/params.c
SRCARM += fusion/altitude_kalman.c
SRCARM += fusion/attitude_observer.c
SRCARM += fusion/position_kalman.c
SRCARM += fusion/vision_buffer.c
SRCARM += fusion/attitude_compl_euler.c
SRCARM += fusion/simple_altitude_moving_average.c
SRCARM += fusion/least_square.c
SRCARM += fusion/global_pos.c
SRCARM += fusion/world_to_body.c
SRCARM += fusion/position_kalman2.c
SRCARM += fusion/position_kalman3.c
SRCARM += math/transformation.c
SRCARM += math/geodetic/latlong.c
SRCARM += arm7/sdfat/syscalls.c
SRCARM += math/geodetic/gps_transformations.c
#SRCARM += arm7/sdfat/mmc_spi.c
#SRCARM += arm7/sdfat/dos.c
#SRCARM += arm7/sdfat/dir.c
#SRCARM += arm7/sdfat/drivefree.c
#SRCARM += arm7/sdfat/find_x.c
#SRCARM += arm7/sdfat/lfn_util.c
#SRCARM += arm7/sdfat/fat.c
#SRCARM += arm7/sdfat/rtc.c
#SRCARM += hal/gps/gps_ubx.c
SRCARM += hal/gps/gps_nmea.c
SRCARM += system/sys_state.c
SRCARM += system/calibration.c
SRCARM += $(TARGET).c

-include user.mk
 
ASRCARM = \
    arm7/include/crt0.S
 
MCU = arm7tdmi
#THUMB = -mthumb
FLOATINGPOINT	= -msoft-fp
THUMB_IW = -mthumb-interwork
FORMAT = ihex
# ENABLE ONCE GCC 4.4.x is broadly used
#DEADCODESTRIP = -Wl,-static -fdata-sections -ffunction-sections -Wl,--gc-sections -Wl,-s
 
# Define programs and commands.
CC     = arm-elf-gcc
LD     = arm-elf-gcc
SHELL = sh
OBJCOPY = arm-elf-objcopy
OBJDUMP = arm-elf-objdump
SIZE = arm-elf-size
NM = arm-elf-nm
REMOVE = rm -f
COPY = cp
 
# Compiler flags.
CFLAGS += -I main
CFLAGS += -I arm7
CFLAGS += -I arm7/include
CFLAGS += -I arm7/spi_devices
CFLAGS += -I arm7/i2c_devices
CFLAGS += -I comm
CFLAGS += -I conf
CFLAGS += -I controllers
CFLAGS += -I controllers/quadrotor
CFLAGS += -I controllers/coaxial
CFLAGS += -I controllers/fixed_wing
CFLAGS += -I fusion 
CFLAGS += -I hal
CFLAGS += -I hal/gps
CFLAGS += -I system
CFLAGS += -I math
CFLAGS += -I math/geodetic
CFLAGS += -I arm7/sdfat
CFLAGS += -I $(MAVLINKBASEDIR)
CFLAGS += -I $(MAVLINKDIR)
	
CFLAGS += -O2
CFLAGS += -Wall -Wcast-qual -Wimplicit -Wcast-align
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -Wa,-adhlns=$(OBJDIR)/$(notdir $(subst $(suffix $<),.lst,$<))
CFLAGS += -Wstrict-prototypes -Wmissing-declarations
CFLAGS += -Wmissing-prototypes -Wnested-externs
CFLAGS += -std=gnu99
 
# Linker flags.
LDFLAGS = -n -nostartfiles -Wl,-Map=$(OBJDIR)/$(TARGET).map,--cref
LDFLAGS += -lc
LDFLAGS += -lm
LDFLAGS += -lc -lgcc
LDSCRIPT = arm7/include/LPC2148-ROM-bl.ld
LDFLAGS +=-T$(LDSCRIPT)
 
# Define all object files.
COBJARM = $(SRCARM:%.c=$(OBJDIR)/%.o)
AOBJARM = $(ASRCARM:%.S=$(OBJDIR)/%.o)
 
# Define all list files
LST = $(SRCARM:%.c=$(OBJDIR)/%.lst)
LST += $(ASRCARM:%.S=$(OBJDIR)/%.lst)
 
# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mcpu=$(MCU) $(THUMB_IW) $(FLOATING_POINT) -I. $(CFLAGS) $(DEADCODESTRIP) $(GENDEPFLAGS)
ALL_ASFLAGS = -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp $(ASFLAGS)
 
# Default target.
all: build size doc
 
#build: generated elf hex lss sym
build: usercheck mavlinkcheck elf hex

usercheck:
	@if test -f conf/user_conf.h; then echo "User config existed, continuing.."; else echo -e "\nNOT COMPILING ANYTHING, PLEASE MAKE USER SETTINGS FIRST:\n\nBecause conf/user_conf.h does not exist, you cannot compile. Most likely you just installed the PIXHAWK codebase. The first step to compile your own code is to COPY conf/user_conf.h.dist to conf/user_conf.h and to edit the file according to your preferences.\n \nPLEASE COPY conf/user_conf.h.dist -> conf/user_conf.h\n" && exit 42; fi

mavlinkcheck:
	@if test -d $(MAVLINKUSERDIR); then echo "Found MAVLink user specific files, using custom messages."; else echo -e "\nNo custom MAVLink header found, using pixhawk message set.\n"; fi

elf: $(OBJDIR)/$(TARGET).elf
hex: $(BINDIR)/$(TARGET).hex
lss: $(OBJDIR)/$(TARGET).lss
sym: $(OBJDIR)/$(TARGET).sym

# Display size of file.
ELFSIZE = $(SIZE) -A $(OBJDIR)/$(TARGET).elf
size:
	@if [ -f $(OBJDIR)/$(TARGET).elf ]; then echo; $(ELFSIZE); echo; fi

doc:
	doxygen doc/Doxyfile
 
# Program the device.
LPC21IAP = $(TOOLS)/lpc21iap/lpc21iap
LPC21IAP-WIN32 = $(TOOLS)/lpc21iap-win32/lpc21iap.exe
bootloader:
  @cd lpc21iap && make --quiet --no-print-directory Q=$(Q)
upload: $(OBJDIR)/$(TARGET).elf bootloader
	@echo ***UPLOAD $(OBJDIR)/$(TARGET).elf
	$(Q)$(LPC21IAP) $(OBJDIR)/$(TARGET).elf
	
upload-win: $(OBJDIR)/$(TARGET).elf bootloader
	@echo ***UPLOAD $(OBJDIR)/$(TARGET).elf
	$(LPC21IAP-WIN32) $(OBJDIR)/$(TARGET).elf

upload-remote: $(OBJDIR)/$(TARGET).elf bootloader 
	$(Q)$(TOOLS)/upload_remote.sh
	
upload-remote-bravo: $(OBJDIR)/$(TARGET).elf bootloader 
	$(Q)$(TOOLS)/upload_remote_bravo.sh
	
upload-remote-charlie: $(OBJDIR)/$(TARGET).elf bootloader 
	$(Q)$(TOOLS)/upload_remote_charlie.sh

# All generated headers
generated: generated/messages.h

generated/messages.h: conf/messages.xml
	@echo GENERATE $@
	$(Q)$(TOOLS)/gen-messages.py $^ > $@

 
# Create final output files (.hex, .eep) from ELF output file.
$(BINDIR)/%.hex: $(OBJDIR)/%.elf
	@echo OBJC $@
	$(Q)$(OBJCOPY) -O $(FORMAT) $< $@
 
# Create extended listing file from ELF output file.
# testing: option -C
$(OBJDIR)/%.lss: $(OBJDIR)/%.elf
	@echo OBJD $@
	$(Q)$(OBJDUMP) -h -S -C $< > $@
 
# Create a symbol table from ELF output file.
$(OBJDIR)/%.sym: $(OBJDIR)/%.elf
	@echo NM $@
	@echo ***no errors
	$(Q)$(NM) -n $< > $@
 
# Link: create ELF output file from object files.
.SECONDARY : $(OBJDIR)/$(TARGET).elf
.PRECIOUS : $(AOBJARM) $(COBJARM)
$(OBJDIR)/$(TARGET).elf: $(AOBJARM) $(COBJARM)
	@echo LD $@
	$(Q)$(CC) $(THUMB) $(ALL_CFLAGS) $(AOBJARM) $(COBJARM) --output $@ $(LDFLAGS)
	@echo ***no errors
# Compile: create object files from C source files. ARM-only
$(OBJDIR)/%.o : %.c
	@echo CC $@
	$(Q)test -d $(dir $@) || mkdir -p $(dir $@)
	$(Q)$(CC) -c $(ALL_CFLAGS) $(CONLYFLAGS) $< -o $@
 
# Assemble: create object files from assembler source files. ARM-only
$(AOBJARM) : $(OBJDIR)/%.o : %.S
	@echo AS $@
	$(Q)test -d $(dir $@) || mkdir -p $(dir $@)
	$(Q)$(CC) -c $(ALL_ASFLAGS) $< -o $@
 
clean:
	$(REMOVE) -r $(BUILDDIR)/
	mkdir -p $(OBJDIR)
	$(REMOVE) -r $(BINDIR)/
	mkdir -p $(BINDIR)

# Listing of phony targets.
.PHONY : all size build elf hex lss sym clean upload
 
