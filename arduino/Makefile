ARDUINOFOLDER	= /opt/arduino
CC		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-gcc
CPP		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-g++
AR		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-ar
OBJCOPY		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-objcopy
AVRDUDE		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avrdude
LIBRARIES	= $(ARDUINOFOLDER)/hardware/arduino/avr/libraries
EXTLIB		= $(ARDUINOFOLDER)/libraries
CFLAGS		= -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=8000000L -DARDUINO=151
CPPFLAGS	= -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=8000000L -DARDUINO=151
INCPATH		= -Iinclude/ -I$(ARDUINOFOLDER)/ -I$(ARDUINOFOLDER)/hardware/arduino/avr/cores/arduino -I$(ARDUINOFOLDER)/hardware/arduino/avr/variants/standard -I$(LIBRARIES)/Wire -I$(LIBRARIES)/Wire/utility -I$(LIBRARIES)/SoftwareSerial -I$(EXTLIB)/CmdArduino -I$(EXTLIB)/Servo
LINK		= $(CPP)
LFLAGS		=
LIBS		= -lm

####### Files

# OBJECTS := $(patsubst %.c,%.o,$(wildcard src/*.c)) $(patsubst %.cpp,%.o,$(wildcard src/*.cpp)) $(patsubst %.cpp,%.o,$(wildcard $(ARDUINOFOLDER)/hardware/arduino/avr/cores/arduino/*.cpp))
OBJECTS := $(patsubst %.c,%.o,$(wildcard src/*.c)) $(patsubst %.cpp,%.o,$(wildcard src/*.cpp))
ARDUINO_CPP_FILES := $(wildcard $(ARDUINOFOLDER)/hardware/arduino/avr/cores/arduino/*.cpp)
ARDUINO_C_FILES := $(wildcard $(ARDUINOFOLDER)/hardware/arduino/avr/cores/arduino/*.c)
ARDUINO_OBJ_FILES := $(addprefix obj/,$(notdir $(ARDUINO_CPP_FILES:.cpp=.o))) $(addprefix obj/,$(notdir $(ARDUINO_C_FILES:.c=.o)))

TARGET := bin/image.hex

####### Implicit rules

.SUFFIXES: .cpp .cxx .cc .C .c

obj/%.o: $(ARDUINOFOLDER)/hardware/arduino/avr/cores/arduino/%.cpp
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o $@ $<

obj/%.o: $(ARDUINOFOLDER)/hardware/arduino/avr/cores/arduino/%.c
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<

.cpp.o:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o $@ $<

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<

####### Build rules
all: $(ARDUINO_OBJ_FILES) $(OBJECTS) twi Wire SoftwareSerial CmdArduino Servo core elf hex

#$(LINK) $(CFLAGS) $(OBJECTS) $(LIBS) -o $(TARGET)

SoftwareSerial:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o obj/SoftwareSerial.o $(LIBRARIES)/SoftwareSerial/SoftwareSerial.cpp

twi:
	$(CC) -c $(CFLAGS) $(INCPATH) -o obj/twi.o $(LIBRARIES)/Wire/utility/twi.c

Wire:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o obj/Wire.o $(LIBRARIES)/Wire/Wire.cpp

Servo:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o obj/Servo.o $(EXTLIB)/Servo/Servo.cpp

CmdArduino:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o obj/CmdArduino.o $(EXTLIB)/CmdArduino/Cmd.cpp

elf:
	$(CC) -Os -Wl,--gc-sections -mmcu=atmega328p -o $(TARGET).elf $(OBJECTS) obj/Wire.o obj/twi.o obj/SoftwareSerial.o obj/CmdArduino.o obj/Servo.o obj/core.a -lm

core: $(ARDUINO_OBJ_FILES)
	$(AR) rcs obj/core.a $^

hex:
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $(TARGET).elf $(TARGET).eep
	$(OBJCOPY) -O ihex -R .eeprom $(TARGET).elf $(TARGET)
clean:
	-rm -f obj/* */*.d bin/* $(OBJECTS) $(TARGET)
	-rm -f *~ core

upload: all
	$(AVRDUDE) -C$(ARDUINOFOLDER)/hardware/tools/avr/etc/avrdude.conf -q -q -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:$(TARGET):i

