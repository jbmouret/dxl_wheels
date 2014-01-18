###############################################################################
# Makefile for the project AX12
###############################################################################

## General Flags
PROJECT = AX12
MCU = atmega8
TARGET = AX12.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99          -DF_CPU=16000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=AX12.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = ax12.o  serial.o  utils.o memory.o motor.o control.o

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) AX12.hex AX12.eep AX12.lss size

## Compile
motor.o: ../motor.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<
serial.o: ../serial.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

utils.o: ../utils.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

control.o: ../control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

memory.o: ../memory.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ax12.o: ../ax12.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<


##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) AX12.elf dep/* AX12.hex AX12.eep AX12.lss AX12.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

