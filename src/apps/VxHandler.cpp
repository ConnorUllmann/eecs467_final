#include "VxHandler.hpp"
#include "a2/CoordinateConverter.hpp"
#include "a2/Arm.hpp"
#include "a2/Constants.hpp"

#include <string>
#include <iostream>

pthread_mutex_t VxHandler::renderMutex;

VxButtonStates buttonStates;
const std::string imageFileName = "data/image.ppm";

VxHandler::VxHandler(int width, int height) :
	windowWidth(width), windowHeight(height) {
	eecs467_init(0, NULL);
	vxWorld = vx_world_create();
	vxeh = (vx_event_handler_t*) calloc(1, sizeof(vx_event_handler_t));
	vxeh->key_event = key_event;
	vxeh->mouse_event = mouse_event;
	vxeh->touch_event = touch_event;
	vxeh->dispatch_order = 0;
	vxeh->impl = this;

	vxApp.display_started = eecs467_default_display_started;
	vxApp.display_finished = eecs467_default_display_finished;
	vxApp.impl = eecs467_default_implementation_create(vxWorld, vxeh);

	if (pthread_mutex_init(&renderMutex, NULL)) {
		printf("renderMutex not initialized\n");
		exit(1);
	}

	buttonStates.colorMask = false;
	buttonStates.blobDetect = false;
	buttonStates.moveArm = false;

	pg = pg_create();
	pg_add_buttons(pg,
		"but1", "Calibrate Masking",
		"but2", "Calibrate Colors",
		"but3", "Calibrate Global Transform",
		"but4", "Save Image",
		"but5", "Pick Up Ball",
		"but9", "Drop Ball",
		"but6", "Coordinate Convert",
		"but7", "Color Mask",
		"but10", "Board Mask",
		"but8", "Blob Detect",
		"but11", "Run",
		NULL);
	pgListener = (parameter_listener_t*) calloc(1, sizeof(parameter_listener_t));
	pgListener->impl = this;
	pgListener->param_changed = parameterEventHandler;
	pg_add_listener(pg, pgListener);
}

VxHandler::~VxHandler() {
	pg_destroy(pg);
	free(vxeh);
}

void VxHandler::launchThreads() {
	pthread_create(&renderPid, NULL, &VxHandler::renderThread, this);
	pthread_create(&mainPid, NULL, &VxHandler::mainThread, this);
}

VxButtonStates VxHandler::getButtonStates() {
	pthread_mutex_lock(&renderMutex);
	VxButtonStates ret = buttonStates;
	pthread_mutex_unlock(&renderMutex);
	return ret;
}

void* VxHandler::renderThread(void* args) {
	VxHandler* state = (VxHandler*) args;

	while (1) {
		pthread_mutex_lock(&state->renderMutex);

		RenderInfo renderInfo = GlobalState::instance()->getData();
		if (renderInfo.im == nullptr) {
			pthread_mutex_unlock(&state->renderMutex);
			continue;
		}

		vx_object_t* vim = vxo_chain(
			vxo_mat_translate3(-0.5, 
				-0.5 * ((float)renderInfo.im->height / renderInfo.im->width), 0),
			vxo_mat_scale(1.0 / renderInfo.im->width),
			vxo_image_from_u32(renderInfo.im, VXO_IMAGE_FLIPY, 0));
		vx_buffer_add_back(vx_world_get_buffer(state->vxWorld, "state"), vim);


		std::string message = CalibrationHandler::instance()->getMessage();
		message = "<<right,#ff00ff,serif>>" + message;
		vx_object_t* vtext = vxo_chain(
			vxo_mat_translate3(400, 40, 0),
			vxo_text_create(VXO_TEXT_ANCHOR_CENTER, message.c_str()));
		vx_buffer_add_back(vx_world_get_buffer(state->vxWorld, "state"), vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vtext));

		if (buttonStates.blobDetect) {
			std::vector<float> redPoints;
			for (auto& blob : renderInfo.redBlobs) {
				redPoints.push_back(blob[0]);
				redPoints.push_back(blob[1]);
				redPoints.push_back(0);
			}
			vx_resc_t* verts = vx_resc_copyf(redPoints.data(), redPoints.size());
			vx_buffer_add_back(vx_world_get_buffer(state->vxWorld, "state"), vxo_points(verts, redPoints.size() / 3, vxo_points_style(vx_red, 5.0f)));			


			std::vector<float> greenPoints;
			for (auto& blob : renderInfo.greenBlobs) {
				greenPoints.push_back(blob[0]);
				greenPoints.push_back(blob[1]);
				greenPoints.push_back(0);
			}
			verts = vx_resc_copyf(greenPoints.data(), greenPoints.size());
			vx_buffer_add_back(vx_world_get_buffer(state->vxWorld, "state"), vxo_points(verts, greenPoints.size() / 3, vxo_points_style(vx_green, 5.0f)));	

			std::vector<float> bluePoints;
			for (auto& blob : renderInfo.blueBlobs) {
				bluePoints.push_back(blob[0]);
				bluePoints.push_back(blob[1]);
				bluePoints.push_back(0);
			}
			verts = vx_resc_copyf(bluePoints.data(), bluePoints.size());
			vx_buffer_add_back(vx_world_get_buffer(state->vxWorld, "state"), vxo_points(verts, bluePoints.size() / 3, vxo_points_style(vx_blue, 5.0f)));
		}

		pthread_mutex_unlock(&state->renderMutex);


		vx_buffer_swap(vx_world_get_buffer(state->vxWorld, "state"));
		usleep(1e3);
	}

	return NULL;
}

void* VxHandler::mainThread(void* args) {
	VxHandler* state = (VxHandler*) args;

	eecs467_gui_run(&state->vxApp, state->pg, 1024, 768);
	return NULL;
}

int VxHandler::mouse_event(vx_event_handler_t *vxeh, vx_layer_t *vl, 
	vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
	VxHandler* state = (VxHandler*) vxeh->impl;
	// vx_camera_pos_t contains camera location, field of view, etc
	// vx_mouse_event_t contains scroll, x/y, and button click events

	RenderInfo renderInfo = GlobalState::instance()->getData();

	if ((mouse->button_mask & VX_BUTTON1_MASK) &&
		!(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {
		vx_ray3_t ray;
		vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

		double ground[3];
		vx_ray3_intersect_xy (&ray, 0, ground);

		CalibrationHandler::instance()->handleMouseEvent(ground[0], 
			ground[1], renderInfo.im);

		pthread_mutex_lock(&renderMutex);
		if (buttonStates.moveArm) {
			std::array<int, 2> imageCoords = 
				CoordinateConverter::screenToImage(std::array<float, 2>{{(float)ground[0], 
					(float)ground[1]}});
			std::array<float, 2> globalCoords = 
				CoordinateConverter::imageToGlobal(imageCoords);

			if (!Arm::instance()->addCommandGrabBall(globalCoords)) {
				printf("Can't move there\n");
			}

			buttonStates.moveArm = false;
		}
		if (buttonStates.dropBall) {
			std::array<int, 2> imageCoords = 
				CoordinateConverter::screenToImage(std::array<float, 2>{{(float)ground[0], 
					(float)ground[1]}});
			std::array<float, 2> globalCoords = 
				CoordinateConverter::imageToGlobal(imageCoords);
			std::array<int, 2> boardCoords = CoordinateConverter::globalToBoard(globalCoords);
			if (!Arm::instance()->addCommandDropBall(boardCoords)) {
				printf("Can't move there\n");
			}

			buttonStates.dropBall = false;
		}
		pthread_mutex_unlock(&renderMutex);

	}

	// store previous mouse event to see if the user *just* clicked or released
	state->last_mouse_event = *mouse;

	return 1;
}

int VxHandler::key_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
	//state_t *state = vxeh->impl;
	return 0;
}

int VxHandler::touch_event(vx_event_handler_t *vh, vx_layer_t *vl, 
	vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
	return 0; // Does nothing
}

void VxHandler::parameterEventHandler (parameter_listener_t *pl, 
	parameter_gui_t *pg, const char *name) {

	pthread_mutex_lock(&renderMutex);
	std::string strName(name);
	if (strName == "but1") {
		// calibrate mask
		if (CalibrationHandler::instance()->isIdle()) {
			CalibrationHandler::instance()->calibrateMask();
		}
	} else if (strName == "but2") {
		// calibrate hsv
		if (CalibrationHandler::instance()->isIdle()) {
			CalibrationHandler::instance()->calibrateHsv();
		}
	} else if (strName == "but3") {
		// calibrate board transform
		if (CalibrationHandler::instance()->isIdle()) {
			CalibrationHandler::instance()->calibrateBoardTransform();
		}
	} else if (strName == "but4") {
		RenderInfo renderInfo = GlobalState::instance()->getData();
		//save image
		int res = image_u32_write_pnm(renderInfo.im, imageFileName.c_str());
		if (res) {
			std::cout << "did not save image successfully\n";
		}
	} else if (strName == "but5") {
		// move arm
		if (CalibrationHandler::instance()->isIdle()) {
			buttonStates.moveArm = true;
		}
	} else if (strName == "but6") {
		// coordinate convert
		if (CalibrationHandler::instance()->isIdle()) {
			CalibrationHandler::instance()->coordTransform();
		}
	} else if (strName == "but7") {
		// color mask
		buttonStates.colorMask = !buttonStates.colorMask;
	} else if (strName == "but8") {
		// blob detect
		buttonStates.blobDetect = !buttonStates.blobDetect;
	} else if (strName == "but9") {
		// drop ball
		if (CalibrationHandler::instance()->isIdle()) {
			buttonStates.dropBall = true;
		}
	} else if (strName == "but10") {
	    // show valid board pixels
	    buttonStates.boardMask = !buttonStates.boardMask;
	} else if (strName == "but11") {
		GlobalState::instance()->setStart(true);
	}
	pthread_mutex_unlock(&renderMutex);
}

