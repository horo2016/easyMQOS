##### Make sure all is the first target.
all:

CXX ?= g++
CC  ?= gcc
#操作开关
CXX_OPTS = -DOPEN_CPUDETECT_PTHREAD #-DWITH_OPENCV #-DMPU_DEBUGOFF  OPEN_STM32_PTHREAD
#是否用远程mqtt server 
#CXX_OPTS += -DMQTT_REMOTE_SERVER 
#是否将终端作为mqtt server 
CXX_OPTS +=-DMQTT_TERMINAL_SERVER
#GPS 的串口号
CXX_OPTS +=-DGPSDEVICE=\"/dev/ttyS5\"
RTIMULIBPATH  = ./RTIMULib

CFLAGS  += -g -pthread -Wall 
CFLAGS  += -rdynamic -funwind-tables

CFLAGS = -O2 

CFLAGS  += -I./Mqtt  -I./inc 
DIR_LIB = lib
DIR_LIBMQTT = Mqtt

DIR_OBJ = obj

DIRS = 	$(DIR_LIB)				\
		$(DIR_LIBMQTT)
		
FILES = $(foreach dir, $(DIRS),$(wildcard $(dir)/*.c))	

SRCS = $(wildcard gps_srcs/*.cpp)  
HEDS = $(wildcard inc/*.h)  
OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(notdir $(SRCS)))  
DEPS = $(patsubst %.o, %.d, $(OBJS))
CFLAGS  += -I./gps_srcs
vpath %.c gps_srcs
CXXDIRS = 	$(RTIMULIBPATH)				\
		    $(RTIMULIBPATH)/IMUDrivers
CXXFILES =	$(foreach dir, $(CXXDIRS),$(wildcard $(dir)/*.cpp))
CXXFLAGS  += -I./inc  

CFLAGS += -D__unused="__attribute__((__unused__))"
CFLAGS +=  $(CXX_OPTS)
LDFLAGS += -ldl
 
LDFLAGS +=  -L./Mqtt/lib -lmosquitto

LDFLAGS += -lrt -lpthread -pthread -lm -ldl 

C_SRC=

#C_SRC+=src/main.c
#C_SRC+=src/mqtt_main.c
C_SRC+=src/cJSON.c
C_SRC+= $(FILES)


CXX_SRC=
CXX_SRC +=src/method_parse.cpp
CXX_SRC +=easy_mqos.cpp
CXX_SRC +=main.cpp
CXX_SRC +=src/geocoords.cpp
CXX_SRC+= $(CXXFILES)
CXX_SRC+=$(SRCS)

OBJ=
DEP=
OBJECTS_DIR   = objects/

# Files
CXXFLAGS += -std=c++11 $(CFLAGS)
#LDFLAGS+= -lcamera

OBJ_CAM_SRV = main.o
TARGETS    += gps_filter_node_pub
$(TARGETS): $(OBJ_CAM_SRV)
TARGET_OBJ += $(OBJ_CAM_SRV)

FILE_LIST := files.txt
COUNT := ./make/count.sh

OBJ=$(CXX_SRC:.cpp=.o) $(C_SRC:.c=.o)
DEP=$(OBJ:.o=.d) $(TARGET_OBJ:.o=.d)

CXXFLAGS += -std=c++11 -g 
CXXFLAGS += -lc -lm -pthread
#include ./common.mk
.PHONY: all clean distclean

all: $(TARGETS)

clean:
	rm -f $(TARGETS) $(FILE_LIST)
	find . -name "*.o" -delete
	find . -name "*.d" -delete

distclean:
	rm -f $(TARGETS) $(FILE_LIST)
	find . -name "*.o" -delete
	find . -name "*.d" -delete

-include $(DEP)

%.o: %.c 
	@[ -f $(COUNT) ] && $(COUNT) $(FILE_LIST) $^ || true
	@$(CC) -c $< -MM -MT $@ -MF $(@:.o=.d) $(CFLAGS) $(LIBQCAM_CFLAGS)
	$(CC) -c $< $(CFLAGS) -o $@ $(LIBQCAM_CFLAGS) $(INCPATH)


%.o: %.cpp 
	@$(CXX) -c $< -MM -MT $@ -MF $(@:.o=.d) $(CXXFLAGS)
	$(CXX) -c $< $(CXXFLAGS) -o $@   $(INCPATH)
	

$(TARGETS): $(OBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)
	@[ -f $(COUNT) -a -n "$(FILES)" ] && $(COUNT) $(FILE_LIST) $(FILES) || true
	@[ -f $(COUNT) ] && $(COUNT) $(FILE_LIST) || true
