# CFLAGS: Compiler flags used to control warnings, optimizations, and architecture settings

# -W: Enable basic warnings to help catch possible issues in your code
# -Wall: Enable all common warnings, like unused variables, potential mistakes, etc.
# -Wextra: Enable additional warnings that might be helpful but aren't part of the default set
# -Werror: Treat all warnings as errors, forcing you to fix them before compiling
# -Wundef: Warn if there are undefined macros (i.e., macros that aren’t defined but are used in the code)
# -Wshadow: Warn if a local variable hides (or "shadows") a global variable with the same name
# -Wdouble-promotion: Warn if a float is automatically promoted to a double, which could be inefficient
# -Wformat-truncation: Warn when a string is too large for the specified format, which could cause data loss
# -fno-common: Prevents the compiler from treating global variables as "common," which could lead to conflicts
# -Wconversion: Warn when implicit type conversions might lose data (like turning a large number into a small one)
# -g3: Include debug information in the output binary (level 3 means a lot of detail, useful for debugging)
# -Os: Optimize the code to make the output smaller (important for systems with limited memory, like embedded devices)
# -ffunction-sections: Put each function in its own section in the output file, so that unused functions can be removed by the linker
# -fdata-sections: Similar to `-ffunction-sections`, but applies this to data variables (again, helping with removing unused data)
# -I.: Tell the compiler to look in the current directory (.) for header files (needed for your project's own headers)

# -mcpu=cortex-m0: Tell the compiler to optimize the code for the ARM Cortex-M0 processor (the one on STM32F072RB)
# -mthumb: Use the Thumb instruction set, which is a more compact version of ARM instructions, saving space
# -mfloat-abi=soft: Use software to handle floating-point operations, because the Cortex-M0 does not have a hardware FPU (Floating Point Unit)

# $(EXTRA_CFLAGS): This is an additional placeholder for any extra flags that may be added later (it's empty by default)

CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m0 -mthumb -mfloat-abi=soft $(EXTRA_CFLAGS)

# LDFLAGS: Linker flags that tell the linker how to link the object files into the final executable

# -Tlink.ld: This flag tells the linker to use a specific linker script (`link.ld`). A linker script helps the linker know how to arrange code and data in memory.
# -nostartfiles: This flag tells the linker **not to use** the standard startup files. Normally, the startup files set up the environment before the main program starts running, but for bare-metal programming, you may not want to use them.
# -nostdlib: This flag tells the linker **not to use the standard C library**. The standard library is a collection of pre-written functions like `printf`, but bare-metal systems often don't use it because they don't need it or have their own custom implementations.
# --specs nano.specs: This flag tells the linker to use a specific set of options that create a **smaller executable**. The `nano.specs` option is commonly used for embedded systems to minimize the size of the output.
# -lc: This tells the linker to include the C standard library (`libc`), even though we’re avoiding the standard library by using `-nostdlib`. This is necessary to handle basic C runtime features, like memory allocation.
# -lgcc: This tells the linker to include the **GCC runtime library**. This library includes support for low-level features, like division and floating-point math, that the standard C library might normally handle.
# -Wl,--gc-sections: This flag tells the linker to **remove unused sections** of code and data (like functions or variables that are never used), which helps reduce the size of the final executable.
# -Wl,-Map=$@.map: This flag tells the linker to generate a **memory map** file (`$@.map`). This file shows how the linker arranged the code and data in memory, and it's useful for debugging or understanding memory usage.

LDFLAGS ?= -Tlinker.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map

# EXTRA_CFLAGS: Additional user-defined compiler flags
# Use this variable to add custom flags when needed (e.g., debugging or optimizations)
EXTRA_CFLAGS ?=

SOURCES = main.c

build: firmware.bin
# Convert the ELF file to a raw binary format using objcopy
# '-O binary' specifies the output format as binary
# $< refers to the first prerequisite (firmware.elf)
# $@ refers to the target file (firmware.bin)
firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

# Use the ARM GCC toolchain to compile the source files into an ELF executable.
# - $(SOURCES) is a list of source files to compile
# - $(CFLAGS) is a set of flags for the compiler (e.g., optimization, warnings)
# - $(LDFLAGS) is a set of flags for the linker (e.g., memory locations)
# - '-o $@' tells the compiler to output the result to 'firmware.elf'
firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

# Flash with J-Link (ST-LINK USB)
jflash: jflash.script
	JLinkExe -commanderscript $<

# Device Configuration (Change this if using a different STM32 variant)
DEVICE = STM32F072RB

jflash.script: firmware.bin
	@echo "device $(DEVICE)" > $@
	@echo "speed 4000" >> $@
	@echo "si SWD" >> $@
	@echo "erase" >> $@       # <-- Erase flash before writing
	@echo "loadbin $< 0x08000000" >> $@
	@echo "r" >> $@            # Reset
	@echo "g" >> $@            # Start execution
	@echo "qc" >> $@           # Quit J-Link

# Flash with ST-Link
stflash: firmware.bin
	st-flash --reset write $< 0x08000000

# Remove any files that start with 'firmware.' (e.g., firmware.elf, firmware.bin)
# 'rm -rf' deletes files and directories recursively
clean:
	rm -rf firmware.* jflash.script

