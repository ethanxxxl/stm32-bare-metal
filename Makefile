################################################################################
#							 CMSIS Core And Device

CMSIS_ROOT_DIR		 = cmsis_core
CMSIS_DEVICE_DIR	 = cmsis_device_f4
INCLUDE_DIRS 		+= -I$(CMSIS_ROOT_DIR)/Include
INCLUDE_DIRS 		+= -I$(CMSIS_DEVICE_DIR)/Include

################################################################################
#								  Object Files
SRC_DIR 		= src
BLD_DIR 		= build

SRC_BASENAMES 	= main system gpio
OBJS 			= $(patsubst %,$(BLD_DIR)/%.o,$(SRC_BASENAMES))

TARGET			= $(BLD_DIR)/panel_firmware

INCLUDE_DIRS 	+= -Iinclude

################################################################################
#								  Device Flags
DEFS		+= -DSTM32F4 -DSTM32F401xE
FP_FLAGS	 = -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 $(FP_FLAGS)

################################################################################
#								 Target CFLAGS

OPTIMIZE 	= -O0
DEBUG 		= -ggdb3 -mgeneral-regs-only
CSTD 		= -std=c11
WARNINGS 	= -Wall -Wextra -Wshadow -pedantic -Werror

TGT_CFLAGS += $(ARCH_FLAGS) $(DEFS)
TGT_CFLAGS += $(OPTIMIZE) $(DEBUG) $(CSTD) $(WARNINGS) $(INCLUDE_DIRS)

###############################################################################
#							  Target Linker Flags

MAPFILE 	 = $(TARGET).map
LIBS 		 = -lc -lgcc -lnosys
LINKERSCRIPT = linkerscript.ld

TGT_LDFLAGS += -T$(LINKERSCRIPT)
TGT_LDFLAGS += -Wl,--cref -Wl,--gc-sections -Wl,-Map=$(MAPFILE)
TGT_LDFLAGS += --static -nostartfiles
TGT_LDFLAGS += $(LIBS) 

################################################################################
#								 Program Names
PREFIX		?= arm-none-eabi-

CC			:= $(PREFIX)gcc
LD			:= $(PREFIX)gcc
AR			:= $(PREFIX)ar
AS			:= $(PREFIX)as
OBJCOPY		:= $(PREFIX)objcopy
OBJDUMP		:= $(PREFIX)objdump
GDB			:= $(PREFIX)gdb

################################################################################
################################################################################
################################################################################

all: elf bin
bin: $(TARGET).bin
elf: $(TARGET).elf
hex: $(TARGET).hex
srec: $(TARGET).srec
list: $(TARGET).list

# Define a helper macro for debugging make errors online
# you can type "make print-OPENCM3_DIR" and it will show you
# how that ended up being resolved by all of the included
# makefiles.
print-%:
	@echo $*=$($*)

%.bin: %.elf
	@echo -e "BIN  :: $@"
	@$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@echo -e "HEX  :: $@"
	@$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@echo -e "SREC :: $@"
	@$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@echo -e "LIST :: $@"
	$(OBJDUMP) -S $(*).elf > $(*).list

%.elf: $(OBJS) Makefile
	@echo -e "LD   :: $@"
	@$(LD) $(TGT_CFLAGS) $(TGT_LDFLAGS) $(OBJS) -o $@

$(BLD_DIR)/%.o: $(SRC_DIR)/%.c
	@echo -e "CC   :: $@"
	@mkdir -p $(@D)
	@$(CC) $(TGT_CFLAGS) $(CFLAGS) -o $@ -c $<

clean:
	$(RM) -r $(BLD_DIR)

.PHONY: images clean elf bin hex srec list
