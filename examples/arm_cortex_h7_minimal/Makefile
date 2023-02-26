LIBS_EMBEDDED_PATH = $(HOME)/programming/libs_embedded/libs/


STD_PERIPH_PATH 	= $(LIBS_EMBEDDED_PATH)/LibsARM/stm32h7/

#MIDDLEWARE_PATH		= $(LIBS_EMBEDDED_PATH)/LibsARM/middleware/
#LIBS_COMMON_PATH	= $(LIBS_EMBEDDED_PATH)/LibsCommon/
#LIBS_SENSORS_PATH	= $(LIBS_EMBEDDED_PATH)/LibsSensors/

BSP_PATH		= $(LIBS_EMBEDDED_PATH)/LibsARM/stm32h7/bsp/ST7735/


GCCPREFIX	=arm-none-eabi-
OUTPUT	 	=bin/build

TARGET 		= -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -fsingle-precision-constant -DSTM32H750xx

LDSCRIPT 	= ./device/ldscript.ld

INC+= -I ./device/
INC+= -I $(STD_PERIPH_PATH)/CMSIS/Include
INC+= -I $(STD_PERIPH_PATH)/CMSIS/Device/ST/STM32H7xx/Include
INC+= -I $(STD_PERIPH_PATH)/Device/ 
INC+= -I $(STD_PERIPH_PATH)/STM32H7xx_HAL_Driver/inc

#INC+= -I $(MIDDLEWARE_PATH) 
#INC+= -I $(LIBS_COMMON_PATH)
#INC+= -I $(LIBS_SENSORS_PATH)

INC+= -include ./device/stm32h7xx_hal_conf.h



INC+= -I .


CXX_SRCS = $(shell find . -name '*.cpp')
#CXX_SRCS+= $(shell find $(MIDDLEWARE_PATH) -name '*.cpp')
#CXX_SRCS+= $(shell find $(LIBS_COMMON_PATH) -name '*.cpp')
#CXX_SRCS+= $(shell find $(LIBS_SENSORS_PATH) -name '*.cpp')

C_SRCS = $(shell find  ./device -name '*.c')
C_SRCS+= $(shell find  $(STD_PERIPH_PATH)/Device/ -name '*.c')
C_SRCS+= $(shell find  $(STD_PERIPH_PATH)/STM32H7xx_HAL_Driver/src -name '*.c')
C_SRCS+= $(shell find  $(BSP_PATH) -name '*.c')


CC 		= $(GCCPREFIX)gcc
CXX 	= $(GCCPREFIX)g++
LD 		= $(GCCPREFIX)g++
OBJCOPY = $(GCCPREFIX)objcopy
OBJDUMP = $(GCCPREFIX)objdump
SIZE 	= $(GCCPREFIX)size


LDFLAGS = $(TARGET) -Wl,--gc-sections

CCFLAGS = $(TARGET) $(INC) -c -O2 -Os  -s -Wall -Wextra -pedantic -std=c99    -fdata-sections -ffunction-sections   -Wl,--gc-sections
CXXFLAGS= $(TARGET) $(INC) -c -O2 -Os  -s -Wall -Wextra -pedantic -std=c++14 -fno-rtti  -fno-exceptions -fdata-sections -ffunction-sections  -Wl,--gc-sections
LDFLAGS = -T$(LDSCRIPT) $(TARGET)  -Wl,--gc-sections -lgcc 


C_OBJS 		= $(C_SRCS:.c=.o)
CXX_OBJS 	= $(CXX_SRCS:.cpp=.o)


all: $(OUTPUT)
	$(OBJDUMP) --disassemble $<.elf > $(OUTPUT).asm
	$(OBJCOPY) -O ihex $<.elf $(OUTPUT).hex
	$(OBJCOPY) -O binary $<.elf $(OUTPUT).bin
	$(SIZE) $(OUTPUT).elf

$(OUTPUT): $(C_OBJS) $(CXX_OBJS) 
	$(LD) -o $@.elf $(C_OBJS) $(CXX_OBJS) $(LDFLAGS)


%.o: %.c
	$(CC) $(CCFLAGS) $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@




CLEAN_FILES = $(C_OBJS)
CLEAN_FILES+= $(CXX_OBJS)
CLEAN_FILES+= $(OUTPUT)*

clean:
	rm $(CLEAN_FILES) -f
