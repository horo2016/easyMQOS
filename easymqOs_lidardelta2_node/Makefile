##### Make sure all is the first target.
all:

CXX ?= g++
CC  ?= gcc
#��������
CXX_OPTS = -DOPEN_CPUDETECT_PTHREAD #-DWITH_OPENCV #-DMPU_DEBUGOFF  OPEN_STM32_PTHREAD
#�Ƿ���Զ��mqtt server 
#CXX_OPTS += -DMQTT_REMOTE_SERVER 
#�Ƿ��ն���Ϊmqtt server 
CXX_OPTS +=-DMQTT_TERMINAL_SERVER
#GPS �Ĵ��ں�
CXX_OPTS +=-DUSBDEVICE=\"/dev/ttyUSB0\"


CFLAGS  += -g -pthread -Wall 
CFLAGS  += -rdynamic -funwind-tables


CFLAGS = -O2 

CFLAGS  += -I./Mqtt  -I./inc 
DIR_LIB = lib

DIR_LIBMQTT = Mqtt
DIR_SRC = src
LIDAR_SRC =LidarDemo/src
DIR_OBJ = obj

DIRS = 	$(DIR_LIB)				\
		$(DIR_LIBMQTT)  \
		$(DIR_SRC)
		
FILES = $(foreach dir, $(DIRS),$(wildcard $(dir)/*.c))	
CXXDIRS = 	$(LIDAR_SRC)  \
			$(DIR_SRC)
			
CXXFILES =	$(foreach dir, $(CXXDIRS),$(wildcard $(dir)/*.cpp))
 
CXXFLAGS  += -I./inc  
CXXFLAGS  += -I./LidarDemo
CXXFLAGS  += -I./LidarDemo/include
CFLAGS += -D__unused="__attribute__((__unused__))"
CFLAGS +=  $(CXX_OPTS)
LDFLAGS += -ldl
 
LDFLAGS +=  -L./Mqtt/lib -lmosquitto

LDFLAGS += -lrt -lpthread -pthread -lm -ldl 

C_SRC=

C_SRC+= $(FILES)


CXX_SRC=

CXX_SRC +=easy_mqos.cpp
CXX_SRC +=main.cpp
CXX_SRC+= $(CXXFILES)
CXX_SRC+=$(SRCS)

OBJ=
DEP=
OBJECTS_DIR   = objects/



CXXFLAGS += -std=c++11 $(CFLAGS)
#LDFLAGS+= -lcamera

OBJ_CAM_SRV = main.o
TARGETS    += lidar_node_pub
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
