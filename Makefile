# export PATH := $(PATH):$(HOME)/local/sdcc/bin

# MAKEFLAGS += --no-builtin-rules
.SUFFIXES:
# .SUFFIXES: .cpp

MCU  = stm8s003f3
ARCH = stm8

F_CPU   ?= 16000000
PROFILE ?= Release
TARGET  ?= iar-stm8-project/$(PROFILE)/Exe/app.out

SRCS    := $(wildcard src/*.cpp src/*.h lib/*.cpp lib/*.h)
ASRCS   := $(wildcard src/*.s lib/*.s)

all: $(TARGET) size

$(TARGET): $(SRCS)
	IarBuild.bat iar-stm8-project/stm8-ntc-temperature-display-DSN_VC288.ewp $(PROFILE)

$(TARGET)-flash.bin: $(TARGET)
	iar-stm8_dump-flash.sh $(TARGET)

$(TARGET)-eeprom.bin: $(TARGET)
	iar-stm8_dump-eeprom.sh $(TARGET)

size: $(TARGET)-flash.bin
	@echo "----------"
	@echo "Image size:"
	@stat -L -c %s $(TARGET)-flash.bin

flash-write: $(TARGET)-flash.bin
	stm8flash -c stlinkv2 -s flash -p $(MCU) -w $<
flash-read:
	stm8flash -c stlinkv2 -s flash -p $(MCU) -r $<
flash-verify:
	stm8flash -c stlinkv2 -s flash -p $(MCU) -v $<

eeprom-write: $(TARGET)-eeprom.bin
	stm8flash -c stlinkv2 -s eeprom -p $(MCU) -w $<
eeprom-read:
	stm8flash -c stlinkv2 -s eeprom -p $(MCU) -r $<
eeprom-verify:
	stm8flash -c stlinkv2 -s eeprom -p $(MCU) -v $<

# serial: $(TARGET).bin
# 	stm8gal -p /dev/ttyUSB0 -w $(TARGET).bin

clean:
	rm -rf iar-stm8-project/{Release,Debug,*.dep}
# 	IarBuild.bat iar-project/blink.ewp -clean Release

.PHONY: clean all size flash-write flash-read eeprom-write eeprom-read

# debugging make
print-%:
	@echo "$*" = "$($*)"
