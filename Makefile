###########################################################################
#   
#  USB smart card device
#   
###########################################################################


# MCU name and submodel
MCU = arm7tdmi
SUBMDL = AT91SAM7S256
#THUMB    = -mthumb
#THUMB_IW = -mthumb-interwork


## Create ROM-Image (final)
RUN_MODE=ROM_RUN
## Create RAM-Image (debugging)
#RUN_MODE=RAM_RUN

# Output format. (can be srec, ihex, binary)
FORMAT = binary

# Target file name (without extension).
TARGET = main
TARGET_NAME = usb_ccid_smartcard

# define the source folders
ASM_SRC_FOLDER = src/asm
C_SRC_FOLDER = src/c
CPP_SRC_FOLDER = src/cpp
LINK_SRC_FOLDER = src/link

# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files
SRC = $(C_SRC_FOLDER)/aic.c $(C_SRC_FOLDER)/flash.c $(C_SRC_FOLDER)/main.c $(C_SRC_FOLDER)/timer.c $(C_SRC_FOLDER)/udp.c
#Cstartup_SAM7.c 
#SRC = 

# List C source files here which must be compiled in ARM-Mode.
# use file-extension c for "c-only"-files
SRCARM = 
#timer.c

# List C++ source files here.
# use file-extension cpp for C++-files (use extension .cpp)
CPPSRC = $(CPP_SRC_FOLDER)/interrupt_utils.cpp

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension cpp for C++-files (use extension .cpp)
#CPPSRCARM = $(TARGET).cpp
CPPSRCARM = 

# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = 

# List Assembler source files here which must be assembled in ARM-Mode..
ASRCARM = $(ASM_SRC_FOLDER)/irq.S $(ASM_SRC_FOLDER)/interrupts.S $(ASM_SRC_FOLDER)/startup_SAM7S.S
#Cstartup.S 

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
#OPT = s
OPT = s

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = dwarf-2
#DEBUG = dwarf-2

# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#EXTRAINCDIRS = ./include
EXTRAINCDIRS = 

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

# Place -D or -U options for C here
CDEFS =  -D$(RUN_MODE) -D$(SUBMDL)

# Place -I options here
CINCS =

# Place -D or -U options for ASM here
ADEFS =  -D$(RUN_MODE)  -D$(SUBMDL)


# Compiler flags.
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += -O$(OPT)
CFLAGS += -Wall -Wcast-align -Wimplicit 
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
#AT91-lib warnings with:
##CFLAGS += -Wcast-qual


# flags only for C
CONLYFLAGS += -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)
#AT91-lib warnings with:
##CONLYFLAGS += -Wmissing-prototypes 
##CONLYFLAGS  = -Wstrict-prototypes
##CONLYFLAGS += -Wmissing-declarations



# flags only for C++ (arm-elf-g++)
# CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS = 

# Assembler flags.
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
##ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 
ASFLAGS = $(ADEFS) -Wa,-adhlns=$(<:.S=.lst),-g$(DEBUG)

#Additional libraries.

#Support for newlibc-lpc (file: libnewlibc-lpc.a)
#NEWLIBLPC = -lnewlib-lpc

MATH_LIB = -lm

# CPLUSPLUS_LIB = -lstdc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -nostartfiles -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -lc
LDFLAGS += $(NEWLIBLPC) $(MATH_LIB)
LDFLAGS += -lc -lgcc 
LDFLAGS += $(CPLUSPLUS_LIB)

# Set Linker-Script Depending On Selected Memory
ifeq ($(RUN_MODE),RAM_RUN)
LDFLAGS +=-Tsrc/link/AT91SAM7S256-RAM.ld
else 
LDFLAGS +=-Tsrc/link/AT91SAM7S256-ROM.ld
endif

# Define programs and commands.
SHELL = sh
CC = arm-elf-gcc
CPP = arm-elf-g++
OBJCOPY = arm-elf-objcopy
OBJDUMP = arm-elf-objdump
SIZE = arm-elf-size
NM = arm-elf-nm
MOVE = C:\MinGW\msys\1.0\bin\mv
#MOVE = mv
REMOVE = rm -f
COPY = cp
RMDIR = rm -r
MKDIR = mkdir


# define the output folder
OUTPUT_BIN_FOLDER = build/bin
OUTPUT_OBJ_FOLDER = build/obj

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_FLASH = Creating load file for Flash:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_ARM = "Compiling C (ARM-only):"
MSG_COMPILINGCPP = Compiling C++:
MSG_COMPILINGCPP_ARM = "Compiling C++ (ARM-only):"
MSG_ASSEMBLING = Assembling:
MSG_ASSEMBLING_ARM = "Assembling (ARM-only):"
MSG_CLEANING = Cleaning project:


# Define all object files.
COBJ      = $(SRC:.c=.o) 
AOBJ      = $(ASRC:.S=.o)
COBJARM   = $(SRCARM:.c=.o)
#CPPOBJARM = $(CPPSRCARM:.cpp=.o)

AOBJARM   = $(ASRCARM:.S=.o)
CPPOBJ    = $(CPPSRC:.cpp=.o) 
CPPOBJARM = $(CPPSRCARM:.cpp=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(ASRCARM:.S=.lst) $(SRC:.c=.lst) $(SRCARM:.c=.lst)
LST += $(CPPSRC:.cpp=.lst) $(CPPSRCARM:.cpp=.lst)

# Compiler flags to generate dependency files.
### GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mcpu=$(MCU) $(THUMB_IW) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp $(ASFLAGS)


# Default target.
all: begin gccversion sizebefore build sizeafter finished end move delete

build: elf hex bin lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex

# meins
bin: $(TARGET).bin

lss: $(TARGET).lss 
sym: $(TARGET).sym

# Eye candy.
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo

move:
	@echo moving files to the bin folder...
	$(RMDIR) $(OUTPUT_BIN_FOLDER)
	$(MKDIR) $(OUTPUT_BIN_FOLDER)
	$(MOVE) $(TARGET).bin $(OUTPUT_BIN_FOLDER)/$(TARGET_NAME).bin
	$(MOVE) $(TARGET).hex $(OUTPUT_BIN_FOLDER)//$(TARGET_NAME).hex
	$(MOVE) $(TARGET).elf $(OUTPUT_BIN_FOLDER)//$(TARGET_NAME).elf

delete:	
	@echo deleting temp files from the source folders...
	$(REMOVE) $(ASM_SRC_FOLDER)/*.o $(ASM_SRC_FOLDER)/*.lst
	$(REMOVE) $(C_SRC_FOLDER)/*.o $(C_SRC_FOLDER)/*.lst
	$(REMOVE) $(CPP_SRC_FOLDER)/*.o $(CPP_SRC_FOLDER)/*.lst
	$(REMOVE) $(TARGET).o $(TARGET).lss $(TARGET).sym $(TARGET).o $(TARGET).lst $(TARGET).map
	
# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
#ELFSIZE = $(SIZE) --target=elf32-bigarm $(TARGET).elf
ELFSIZE = echo OK
sizebefore:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi


# Display compiler version information.
gccversion : 
	@$(CC) --version


# Program the device.  
program: $(TARGET).hex
	@echo
	@echo $(MSG_LPC21_RESETREMINDER)
	$(LPC21ISP) $(LPC21ISP_CONTROL) $(LPC21ISP_DEBUG) $(LPC21ISP_FLASHFILE) $(LPC21ISP_PORT) $(LPC21ISP_BAUD) $(LPC21ISP_XTAL)

# meins :-)
%.bin: %.hex
	@echo
	@echo copying hex $< to bin $@
	$(COPY) $< $@
	

# Create final output files (.hex, .eep) from ELF output file.
# TODO: handling the .eeprom-section should be redundant
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) $< $@


# Create extended listing file from ELF output file.
# testing: option -C
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C $< > $@


# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@


# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM)
%.elf:  $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(THUMB) $(ALL_CFLAGS) $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM) --output $@ $(LDFLAGS)
#	$(CPP) $(THUMB) $(ALL_CFLAGS) $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM) --output $@ $(LDFLAGS)

# Compile: create object files from C source files. ARM/Thumb
$(COBJ) : %.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(THUMB) $(ALL_CFLAGS) $(CONLYFLAGS) $< -o $@ 

# Compile: create object files from C source files. ARM-only
$(COBJARM) : %.o : %.c
	@echo
	@echo $(MSG_COMPILING_ARM) $<
	$(CC) -c $(ALL_CFLAGS) $(CONLYFLAGS) $< -o $@ 

# Compile: create object files from C++ source files. ARM/Thumb
$(CPPOBJ) : %.o : %.cpp
	@echo
	@echo $(MSG_COMPILINGCPP) $<
	$(CPP) -c $(THUMB) $(ALL_CFLAGS) $(CPPFLAGS) $< -o $@ 

# Compile: create object files from C++ source files. ARM-only
$(CPPOBJARM) : %.o : %.cpp
	@echo
	@echo $(MSG_COMPILINGCPP_ARM) $<
	$(CPP) -c $(ALL_CFLAGS) $(CPPFLAGS) $< -o $@ 


# Compile: create assembler files from C source files. ARM/Thumb
## does not work - TODO - hints welcome
##$(COBJ) : %.s : %.c
##	$(CC) $(THUMB) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files. ARM/Thumb
$(AOBJ) : %.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(THUMB) $(ALL_ASFLAGS) $< -o $@


# Assemble: create object files from assembler source files. ARM-only
$(AOBJARM) : %.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING_ARM) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@


# Target: clean project.
clean: begin clean_list finished end


clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(COBJ)
	$(REMOVE) $(CPPOBJ)
	$(REMOVE) $(AOBJ)
	$(REMOVE) $(COBJARM)
	$(REMOVE) $(CPPOBJARM)
	$(REMOVE) $(AOBJARM)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(SRCARM:.c=.d)
	$(REMOVE) $(CPPSRC:.cpp=.s) 
	$(REMOVE) $(CPPSRC:.cpp=.d)
	$(REMOVE) $(CPPSRCARM:.cpp=.s) 
	$(REMOVE) $(CPPSRCARM:.cpp=.d)
	$(REMOVE) .dep/*


# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex lss sym clean clean_list program

