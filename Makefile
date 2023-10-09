#define variables

OPENCM3_DIR := ./libopencm3
WHAD_DIR 	:= ./whad-lib
LIBDIR       = $(OPENCM3_DIR)/lib
LIBNAMEROOT  = opencm3_
DEVICE	     = stm32wl

DEV_DATAFILE = $(OPENCM3_DIR)/ld/devices.data
DEV_LDFILE   = $(DEVICE).ld

DEV_FAMILY    = $(strip $(shell $(OPENCM3_DIR)/scripts/genlink.py $(DEV_DATAFILE) $(DEVICE) FAMILY))
DEV_SUBFAMILY = $(strip $(shell $(OPENCM3_DIR)/scripts/genlink.py $(DEV_DATAFILE) $(DEVICE) SUBFAMILY))
DEV_CPU       = $(strip $(shell $(OPENCM3_DIR)/scripts/genlink.py $(DEV_DATAFILE) $(DEVICE) CPU))
DEV_FPU       = $(strip $(shell $(OPENCM3_DIR)/scripts/genlink.py $(DEV_DATAFILE) $(DEVICE) FPU))
DEV_LDDATA    = $(strip $(shell $(OPENCM3_DIR)/scripts/genlink.py $(DEV_DATAFILE) $(DEVICE) DEFS))
DEV_CFLAGS    = $(strip $(shell $(OPENCM3_DIR)/scripts/genlink.py $(DEV_DATAFILE) $(DEVICE) CPPFLAGS))
DEV_FAMILYCODE= $(strip $(shell echo $(DEV_FAMILY)|sed 's/stm32//'))
DEV_FLOATTYPE = $(strip $(shell echo $(DEV_FPU)|sed 's/-.*//'))
DEV_FPUTYPE   = $(strip $(shell echo $(DEV_FPU)|sed 's/[^-]*-//'))
DEV_ROMOFF    = $(strip $(shell echo $(DEV_LDDATA)|sed -r 's/.*?ROM_OFF=(\S+)\s.*/\1/'))


LIBNAME	     = $(LIBNAMEROOT)$(DEV_FAMILY)

#for the tools
PREFIX		?= arm-none-eabi
CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
SIZE		:= $(PREFIX)-size
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB	        := gdb-multiarch
STFLASH         := st-flash

#for the compilation
ifeq ($(DEV_FLOATTYPE),soft)
C_FLAGS	     = -Os -mthumb -mcpu=$(DEV_CPU) $(DEV_CFLAGS) -Wall 
else
C_FLAGS	     = -Os -mthumb -mcpu=$(DEV_CPU) $(DEV_CFLAGS) -m$(DEV_FLOATTYPE)-float -mfpu=$(DEV_FPUTYPE) -Wall
endif


INC_FLAGS    += -I$(OPENCM3_DIR)/include -I$(WHAD_DIR)/inc -I$(WHAD_DIR)/nanopb -I$(WHAD_DIR)

#for the linking
LD_FLAGS     += -L$(OPENCM3_DIR)/lib -lc -lgcc -lnosys -l$(LIBNAME) -T$(DEV_LDFILE) -nostartfiles --static
LD_FLAGS     += -L$(WHAD_DIR)/lib -lwhad
LD_FLAGS     += -u _printf_float

PROJECT      = main
ADAPTER		 = adapter
PPACKET		 = ppacket

# tells make that these don't produce a file
.PHONY: clean flash lib $(PROJECT)

$(PROJECT) : $(PROJECT).elf $(PROJECT).bin $(PROJECT).hex ;

flash : $(PROJECT).bin	
	$(STFLASH) write $(PROJECT).bin $(DEV_ROMOFF)

lib:
	$(MAKE) -C $(OPENCM3_DIR) DEVICE=$(DEVICE) TARGETS=stm32/$(DEV_FAMILYCODE) -j `nproc`
	$(MAKE) -C $(WHAD_DIR) ARCH_ARM=1 all

clean :
	@rm *.bin *.hex *.elf *.o *.ld lora/*.o

ultraclean:
	$(MAKE) -C $(OPENCM3_DIR) clean
	@rm *.bin *.hex *.elf *.o

$(DEV_LDFILE):
	$(CC) $(OPENCM3_DIR)/ld/linker.ld.S $(DEV_LDDATA) -P -E -o $(DEV_LDFILE)

%.bin: %.elf
	$(OBJCOPY) -Obinary $(PROJECT).elf $(PROJECT).bin

%.hex: %.elf
	$(OBJCOPY) -Oihex $(PROJECT).elf $(PROJECT).hex

$(PROJECT).elf : lib $(PROJECT).o $(ADAPTER).o $(PPACKET).o $(DEV_LDFILE) lora/subghz.o
	$(CC) $(C_FLAGS) -o $(PROJECT).elf $(PROJECT).o $(ADAPTER).o $(PPACKET).o lora/subghz.o $(LD_FLAGS)

lora/subghz.o: lora/subghz.c
	$(CC) $(C_FLAGS) $(INC_FLAGS) -c -o lora/subghz.o lora/subghz.c

$(PPACKET).o: $(PPACKET).c
	$(CC) $(C_FLAGS) $(INC_FLAGS) -c -o $(PPACKET).o $(PPACKET).c
	
$(PROJECT).o : $(PROJECT).c
	$(CC) $(C_FLAGS) $(INC_FLAGS) -c -o $(PROJECT).o $(PROJECT).c

$(ADAPTER).o : $(ADAPTER).c
	$(CC) $(C_FLAGS) $(INC_FLAGS) -c -o $(ADAPTER).o $(ADAPTER).c

makefiledebug:
	@echo "DEV_FAMILY='"$(DEV_FAMILY)"'"
	@echo "DEV_SUBFAMILY='"$(DEV_SUBFAMILY)"'"
	@echo "DEV_FAMILYCODE='"$(DEV_FAMILYCODE)"'"
	@echo "DEV_CPU='"$(DEV_CPU)"'"
	@echo "DEV_FPU='"$(DEV_FPU)"'"
	@echo "DEV_LDDATA='"$(DEV_LDDATA)"'"
	@echo "DEV_CFLAGS='"$(DEV_CFLAGS)"'"
	@echo "DEV_FLOATTYPE='"$(DEV_FLOATTYPE)"'"
	@echo "DEV_FPUTYPE='"$(DEV_FPUTYPE)"'"
	@echo "DEV_ROMOFF='"$(DEV_ROMOFF)"'"
	@echo "C_FLAGS='"$(C_FLAGS)"'"
	@echo "LD_FLAGS='"$(LD_FLAGS)"'"
	@echo "INC_FLAGS='"$(INC_FLAGS)"'"
