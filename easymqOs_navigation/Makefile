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
CXX_OPTS +=-DSTMDEVICE=\"/dev/ttystm\"
 
ARCH = $(shell uname -m)

CFLAGS  += -g -pthread -Wall 
CFLAGS  += -rdynamic -funwind-tables

INCPATH   += -I. 
CFLAGS = -O2 

CFLAGS  +=  -I.  -I./Mqtt   -I./inc -I/usr/local/include/eigen3
DIR_LIB = lib
DIR_LIBMQTT = Mqtt
DIR_SRC = src

DIR_OBJ = obj

DIRS = 	$(DIR_LIB)				\
		$(DIR_LIBMQTT)  \
		$(DIR_SRC)
		
FILES = $(foreach dir, $(DIRS),$(wildcard $(dir)/*.c))	



		    
CXXDIRS = 			$(DIR_SRC)
CXXFILES =	$(foreach dir, $(CXXDIRS),$(wildcard $(dir)/*.cpp))
 
CXXFLAGS  += -I./inc      

CFLAGS += -D__unused="__attribute__((__unused__))"
CFLAGS +=  $(CXX_OPTS)
CFLAGS += -I/usr/local/include/opencv
CFLAGS += -I/usr/inc/opencv/opencv2
CFLAGS += -I/usr/local/include 
LDFLAGS += -ldl
 
LDFLAGS +=   -lmosquitto
LDFLAGS += -lopencv_core -ldl -lm  -lstdc++

LDFLAGS +=  -lopencv_calib3d   -lopencv_features2d   -lopencv_imgcodecs   -lopencv_ml  -lopencv_objdetect  -lopencv_photo  
LDFLAGS +=  -lopencv_shape -lopencv_stitching -lopencv_superres  -lopencv_video -lopencv_videostab -lopencv_videoio   -lopencv_highgui
LDFLAGS += #-lIlmImf -llibjasper -llibtiff  
LDFLAGS += -llibjpeg -llibpng  -lopencv_imgproc -lopencv_flann -lopencv_core
LDFLAGS += -lrt -lpthread -pthread -lm -ldl -lz

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

# Files
CXXFLAGS += -std=c++11 $(CFLAGS)
#LDFLAGS+= -lcamera

OBJ_CAM_SRV = main.o
TARGETS    += navigation_node
$(TARGETS): $(OBJ_CAM_SRV)
TARGET_OBJ += $(OBJ_CAM_SRV)

FILE_LIST := files.txt
COUNT := ./make/count.sh

OBJ=$(CXX_SRC:.cpp=.o) $(C_SRC:.c=.o)
DEP=$(OBJ:.o=.d) $(TARGET_OBJ:.o=.d)
CXXFLAGS +=  $(CFLAGS)

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
