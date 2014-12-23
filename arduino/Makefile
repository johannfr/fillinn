ARDUINOFOLDER	= /opt/arduino
CC		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-gcc
CPP		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-g++
AR		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-ar
OBJCOPY		= $(ARDUINOFOLDER)/hardware/tools/avr/bin/avr-objcopy
AVRDUDE		= $(ARDUINOFOLDER)/hardware/tools/avrdude
LIBRARIES	= $(ARDUINOFOLDER)/hardware/arduino/avr/libraries
EXTLIB		= $(ARDUINOFOLDER)/libraries
CFLAGS		= -g -Os -w -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=8000000L -DARDUINO=151
CPPFLAGS	= -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p -DF_CPU=8000000L -DARDUINO=151
INCPATH		= -Iinclude/ -I$(ARDUINOFOLDER)/ -I$(ARDUINOFOLDER)/hardware/arduino/avr/cores/arduino -I$(ARDUINOFOLDER)/hardware/arduino/avr/variants/standard -I$(LIBRARIES)/SoftwareSerial -I$(EXTLIB)/PID_v1 -I$(EXTLIB)/PID_AutoTune_v0
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
all: $(ARDUINO_OBJ_FILES) $(OBJECTS) SoftwareSerial ArduinoPid ArduinoAutoTune core elf hex

#$(LINK) $(CFLAGS) $(OBJECTS) $(LIBS) -o $(TARGET)

SoftwareSerial:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o obj/SoftwareSerial.o $(LIBRARIES)/SoftwareSerial/SoftwareSerial.cpp

ArduinoPid:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o obj/PID_v1.o $(EXTLIB)/PID_v1/PID_v1.cpp

ArduinoAutoTune:
	$(CPP) -c $(CPPFLAGS) $(INCPATH) -o obj/PID_AutoTune_v0.o $(EXTLIB)/PID_AutoTune_v0/PID_AutoTune_v0.cpp	

elf:
	$(CC) -Os -Wl,--gc-sections -mmcu=atmega328p -o $(TARGET).elf $(OBJECTS) obj/SoftwareSerial.o obj/PID_v1.o obj/PID_AutoTune_v0.o obj/core.a -lm

core: $(ARDUINO_OBJ_FILES)
	$(AR) rcs obj/core.a $^

hex:
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $(TARGET).elf $(TARGET).eep
	$(OBJCOPY) -O ihex -R .eeprom $(TARGET).elf $(TARGET)
clean:
	-rm -f obj/* */*.d bin/* $(OBJECTS) $(TARGET)
	-rm -f *~ core

upload: all
	$(AVRDUDE) -C$(ARDUINOFOLDER)/hardware/tools/avrdude.conf -q -q -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:$(TARGET):i

