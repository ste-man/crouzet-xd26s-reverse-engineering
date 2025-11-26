PROGRAM_NAME = crouzet-xd26s
SRC_DIR = src
BUILD_DIR = build

all: $(PROGRAM_NAME)

# Create the build directory
$(BUILD_DIR):
	mkdir -pv $(BUILD_DIR)

# Compile and build the program for AtTiny85
$(PROGRAM_NAME): $(BUILD_DIR)
	avr-gcc -mmcu=atmega128 -Wall -Og -o $(BUILD_DIR)/$(PROGRAM_NAME).elf $(SRC_DIR)/$(PROGRAM_NAME).c
	avr-objcopy -j .text -j .data -O ihex $(BUILD_DIR)/$(PROGRAM_NAME).elf $(BUILD_DIR)/$(PROGRAM_NAME).hex

# Upload the built program (hex file) to ATmega128 using USBasp
upload: $(PROGRAM_NAME)
#	avrdude -c avrispmkII -p m128 -U flash:w:$(BUILD_DIR)/$(PROGRAM_NAME).hex
#	avrdude -c atmelice_isp -p m128 -U flash:w:$(BUILD_DIR)/$(PROGRAM_NAME).hex
	avrdude -c dragon_jtag -p m128 -U flash:w:$(BUILD_DIR)/$(PROGRAM_NAME).hex

# Remove build directory with all built files
clean:
	rm -rf $(BUILD_DIR)
