PROJECT = MineCar

# Imported source files and paths
CONFDIR = ./config
COREDIR = ./internal-core
BOARD_NAME = STM32F103x8
USE_FPU = no

include ./board/board.mk

# Compile all .c and .cpp files in the root directory
ALLCSRC += $(wildcard ./*.c)
ALLCPPSRC += $(wildcard ./*.cpp)

include $(COREDIR)/core.mk
