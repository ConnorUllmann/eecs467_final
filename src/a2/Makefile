include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) -O3 -DNDEBUG
CXXFLAGS = $(CXXFLAGS_STD) -O3 -DNDEBUG
LDFLAGS = $(LDFLAGS_STD) 

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_A2 = $(LIB_PATH)/liba2.a
LIB_A2_OBJS = CalibrationHandler.o \
	CoordinateConverter.o ColorRecognizer.o \
	Board.o BlobDetector.o LcmHandler.o \
	Arm.o GamePlayer.o \
	GlobalState.o

ALL = $(LIB_A2)

all: $(ALL)

$(LIB_A2): $(LIB_A2_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
