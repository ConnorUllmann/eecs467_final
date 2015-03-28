#ifndef VX_HANDLER_HPP
#define VX_HANDLER_HPP

#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

#include "a2/CalibrationHandler.hpp"
#include "a2/CalibrationInfo.hpp"
#include "a2/GlobalState.hpp"

#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

#include "eecs467_util.h"

#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"

#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"


struct VxButtonStates {
	bool colorMask;
	bool blobDetect;
	bool moveArm;
	bool dropBall;
	bool boardMask;
};

extern VxButtonStates buttonStates;

// you can really only have one instance of VxHandler
// it's just in a class because it looks nice
class VxHandler {
private:
	// vx stuff
	vx_application_t vxApp;
	vx_world_t* vxWorld;      // where vx objects are live
	vx_event_handler_t* vxeh; // for getting mouse, key, and touch events

	vx_mouse_event_t last_mouse_event;

	static pthread_mutex_t renderMutex;
	pthread_t renderPid;
	pthread_t mainPid;

	parameter_gui_t* pg;
	parameter_listener_t* pgListener;

	int windowWidth, windowHeight;
	CalibrationInfo info;

public:
	VxHandler(int width, int height);
	~VxHandler();

	void launchThreads();

	VxButtonStates getButtonStates();

private:
	static void* renderThread(void* args);

	static void* mainThread(void* args);

	static void display_finished(vx_application_t * app, vx_display_t * disp);

	static void display_started(vx_application_t * app, vx_display_t * disp);

	static void parameterEventHandler (parameter_listener_t *pl, parameter_gui_t *pg, const char *name);

	static int mouse_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse);

	static int key_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key);

	static int touch_event(vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse);
};


#endif /* VX_HANDLER_HPP */
