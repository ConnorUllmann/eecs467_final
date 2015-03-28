#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

#include <iostream>

#include "VxHandler.hpp"
#include "a2/ColorRecognizer.hpp"
#include "a2/CoordinateConverter.hpp"
#include "a2/BlobDetector.hpp"
#include "a2/Constants.hpp"
#include "a2/LcmHandler.hpp"
#include "a2/Arm.hpp"
#include "a2/GlobalState.hpp"
#include "a2/GamePlayer.hpp"

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"

//lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"
#include "lcmtypes/ttt_turn_t.hpp"

#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"


class CameraHandler {
private:
	image_source_t* _isrc;
	image_u32_t* _im;
	pthread_t _captureThreadPid;
	mutable pthread_mutex_t _dataMutex;
	bool _running;
	bool _staticImage;

public:
	CameraHandler() {
		zarray_t* urls = image_source_enumerate();

		bool gotCamera = false;
		for (int i = 0; i < zarray_size(urls); ++i) {
			char* url;
			zarray_get(urls, i, &url);
			_isrc = image_source_open(url);
			if (_isrc != NULL) {
				printf("connected to camera %s\n", url);
				gotCamera = true;
				free(url);
				break;
			}
		}
		zarray_destroy(urls);
		if (!gotCamera) {
			printf("couldn't find a camera\n");
			exit(1);
		}

		if (pthread_mutex_init(&_dataMutex, NULL)) {
			printf("dataMutex not initialized\n");
			exit(1);
		}

		_im = nullptr;
		_running = false;
		_staticImage = false;
		_isrc->start(_isrc);
	}

	~CameraHandler() {
		if (_im != nullptr) {
			image_u32_destroy(_im);
		}
		_isrc->close(_isrc);
		pthread_mutex_destroy(&_dataMutex);
	}

	image_u32_t* getImage() {
		if (_staticImage) {
			image_u32_t* ret = image_u32_copy(_im);
			return ret;
		}
		if (!_running) {
			image_source_data_t isData;
			int res = _isrc->get_frame(_isrc,
				&isData);
			if (!res) {
				pthread_mutex_lock(&_dataMutex);
				image_u32_destroy(_im);
				_im = image_convert_u32(&isData);
				pthread_mutex_unlock(&_dataMutex);
			}
			_isrc->release_frame(_isrc,
				&isData);
		}

		image_u32_t* ret;
		pthread_mutex_lock(&_dataMutex);
		if (_im == nullptr) {
			ret = image_u32_create(1, 1);
			int size = ret->height * ret->stride;
			for (int i = 0; i < size; ++i) {
				ret->buf[i] = 0;
			}
		} else {
			ret = image_u32_create(_im->width, _im->height);
			int size = ret->height * ret->stride;
			for (int i = 0; i < size; ++i) {
				ret->buf[i] = _im->buf[i];
			}
		}
		pthread_mutex_unlock(&_dataMutex);
		return ret;
	}

	void setStaticImage(const char* fileName) {
		pthread_mutex_lock(&_dataMutex);
		_staticImage = true;
		_im = image_u32_create_from_pnm(fileName);
		pthread_mutex_unlock(&_dataMutex);
	}

	void launchThreads() {
		pthread_create(&_captureThreadPid, NULL, 
			&CameraHandler::captureThread, this);
		_running = true;
	}

private:
	static void* captureThread(void* args) {
		CameraHandler* state = (CameraHandler*) args;
		while (1) {
			image_source_data_t isData;
			int res = state->_isrc->get_frame(state->_isrc,
				&isData);
			if (!res) {
				pthread_mutex_lock(&state->_dataMutex);
				image_u32_destroy(state->_im);
				state->_im = image_convert_u32(&isData);
				pthread_mutex_unlock(&state->_dataMutex);
			}
			state->_isrc->release_frame(state->_isrc,
				&isData);
					}
		return NULL;
	}
};

int main(int argc, char** argv)
{
	// camera
	CameraHandler camera;

	// getopt
	getopt_t* gopt = getopt_create();
	getopt_add_string(gopt, 'f', "file", "", "Use static camera image");
	getopt_add_string(gopt, 'c', "color", "red", "Specify what color team you are: \"red\" or \"green\"");
	getopt_add_bool(gopt, 'r', "red", false, "use this option to specify red team");
	getopt_add_bool(gopt, 'g', "green", false, "use this option to specify green team");

	if (!getopt_parse(gopt, argc, argv, 1)) {
		getopt_do_usage(gopt);
		exit(1);
	}
	const char* fileName = getopt_get_string(gopt, "file");
	if (strncmp(fileName, "", 1)) {
		// if fileName is not empty
		camera.setStaticImage(fileName);
	}

	// initialize with first image
	image_u32_t* tempIm = camera.getImage();
	CalibrationHandler::instance()->calibrateImageSize(tempIm->height, tempIm->width, true);
	image_u32_destroy(tempIm);

	OBJECT color;
	bool redTeam = getopt_get_bool(gopt, "red");
	bool greenTeam = getopt_get_bool(gopt, "green");
	if ((!redTeam && !greenTeam) || (redTeam && greenTeam)) {
		printf("use at least one of -r or -g but not both\n");
		exit(1);
	}
	if (redTeam) {
		printf("You are RED team\n");
		color = REDBALL;
	} else {
		printf("You are GREEN team\n");
		color = GREENBALL;
	}
	
	GamePlayer::instance()->init(color);
	GamePlayer::instance()->launchThreads();

	// lcm
	LcmHandler::instance()->launchThreads();
	if(redTeam)
		LcmHandler::instance()->setOpponentColor("GREEN_TURN");
	else
		LcmHandler::instance()->setOpponentColor("RED_TURN");
	// vx
	VxHandler vx(1024, 768);
	vx.launchThreads();
	Arm::instance()->addHomeCommand(0);
	while (1) {
		CalibrationInfo calibrationInfo = 
			CalibrationHandler::instance()->getCalibration();
		RenderInfo render;
		render.im = camera.getImage();
		CalibrationHandler::instance()->clipImage(render.im);
/*
		std::array<float, 2> pos;
		if (Arm::instance()->forwardKinematics(pos)) {
			printf("pos: %f, %f\n", pos[0], pos[1]);			
		}
*/
		if (buttonStates.blobDetect) {
			std::vector<BlobDetector::Blob> blobs = 
				BlobDetector::findBlobs(render.im, calibrationInfo, blobMinPixels);
			for (const auto& blob : blobs) {
				std::array<int, 2> imageCoords{{blob.x, blob.y}};
				std::array<float, 2> screenCoords = 
					CoordinateConverter::imageToScreen(imageCoords);
				switch (blob.type) {
					case REDBALL:
						render.redBlobs.push_back(screenCoords);
						break;
					case GREENBALL:
						render.greenBlobs.push_back(screenCoords);
						break;
					case BLUESQUARE:
						render.blueBlobs.push_back(screenCoords);
						break;
					default:
						break;
				}
			}
		}

		VxButtonStates buttonStates = vx.getButtonStates();
		if (buttonStates.colorMask) {
			maskWithColors(render.im, calibrationInfo);
		}
		if (buttonStates.boardMask) {
			maskWithBoard(render.im, calibrationInfo);
		}

		GlobalState::instance()->setData(render);
		
		usleep(1e3);
	}

	getopt_destroy(gopt);
}

