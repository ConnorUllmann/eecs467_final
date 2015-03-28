include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_A2) \
	$(CFLAGS_GSL) \
	-O2

LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	$(LDFLAGS_IMAGESOURCE) \
	$(LDFLAGS_MATH) \
	$(LDFLAGS_COMMON) \
	$(LDFLAGS_LCMTYPES) \
	$(LDFLAGS_MAPPING) \
	$(LDFLAGS_A2) \
	$(LDFLAGS_GSL)

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_A2) \
	$(CFLAGS_GSL) \
	-O2

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o

BIN_EECS467_GUI_EXAMPLE = $(BIN_PATH)/eecs467_gui_example
BIN_EECS467_REXARM_EXAMPLE = $(BIN_PATH)/eecs467_rexarm_example
BIN_EECS467_MAEBOT_EXAMPLE = $(BIN_PATH)/eecs467_maebot_example
BIN_EECS467_MAEBOT_VIDEO_TELOP = $(BIN_PATH)/eecs467_maebot_video_telop
BIN_EECS467_VX_REXARM_DEMO = $(BIN_PATH)/vx_rexarm_demo
BIN_EECS467_REXARM_MAIN = $(BIN_PATH)/eecs467_rexarm_main
BIN_EECS467_BLOB_TEST = $(BIN_PATH)/eecs467_blob_test
BIN_EECS467_ARM_TEST = $(BIN_PATH)/eecs467_arm_test
BIN_EECS467_LCM_TEST = $(BIN_PATH)/eecs467_lcm_test

ALL = $(LIB_EECS467) \
	$(BIN_EECS467_GUI_EXAMPLE) \
	$(BIN_EECS467_REXARM_EXAMPLE) \
	$(BIN_EECS467_MAEBOT_EXAMPLE) \
	$(BIN_EECS467_MAEBOT_VIDEO_TELOP) \
	$(BIN_EECS467_VX_REXARM_DEMO) \
	$(BIN_EECS467_REXARM_MAIN) \
    	$(BIN_EECS467_BLOB_TEST) \
    	$(BIN_EECS467_ARM_TEST) \
 	$(BIN_EECS467_LCM_TEST)


all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_EECS467_GUI_EXAMPLE): gui_example.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_EECS467_REXARM_EXAMPLE): rexarm_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_EXAMPLE): maebot_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_VIDEO_TELOP): maebot_video_teleop.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_VX_REXARM_DEMO): vx_rexarm_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_REXARM_MAIN): eecs467_rexarm_main.o VxHandler.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_BLOB_TEST): blobtest.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_ARM_TEST): arm_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_LCM_TEST): lcm_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
