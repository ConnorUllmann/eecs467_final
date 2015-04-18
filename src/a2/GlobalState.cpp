#include "GlobalState.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

GlobalState* GlobalState::_instance = new GlobalState;


RenderInfo::RenderInfo() : im(nullptr), start(false) {}

RenderInfo::~RenderInfo() {
    if (im) {
	   image_u32_destroy(im);
    }
}

void RenderInfo::copy(const RenderInfo& info) {
	redBlobs = info.redBlobs;
	greenBlobs = info.greenBlobs;
	blueBlobs = info.blueBlobs;
	start = info.start;
    utime = info.utime;
    // manual = info.manual;

	if (info.im == nullptr) {
		return;
	}

	if (im == nullptr) {
		im = image_u32_create(info.im->width, info.im->height);
	}
	if (info.im->stride * info.im->height != im->stride * im->height) {
		image_u32_destroy(im);
		im = image_u32_create(info.im->width, info.im->height);
	}
	int size = im->stride * im->height;
	memcpy(im->buf, info.im->buf, size * sizeof(uint32_t));
}

void RenderInfo::operator=(const RenderInfo& info) {
    copy(info);
    /*
	if (im == nullptr) {
		im = image_u32_create(info.im->width, info.im->height);
	}
	if (info.im->stride * info.im->height != im->stride * im->height) {
		image_u32_destroy(im);
		im = image_u32_create(info.im->width, info.im->height);
	}
	int size = im->stride * im->height;
	memcpy(im->buf, info.im->buf, size * sizeof(uint32_t));

	redBlobs = info.redBlobs;
	greenBlobs = info.greenBlobs;
	blueBlobs = info.blueBlobs;
    start = info.start;
    utime = info.utime;
    */
}

GlobalState::GlobalState() {
	if (pthread_mutex_init(&_dataMutex, NULL)) {
		printf("dataMutex not initialized\n");
		exit(1);
	}
}

GlobalState* GlobalState::instance() {
	return _instance;
}

void GlobalState::setData(const RenderInfo& data) {
	pthread_mutex_lock(&_dataMutex);
	_data = data;
	pthread_mutex_unlock(&_dataMutex);
}

RenderInfo GlobalState::getData() {
	pthread_mutex_lock(&_dataMutex);
	RenderInfo ret;
	ret.copy(_data);
	pthread_mutex_unlock(&_dataMutex);
	return ret;
}

void GlobalState::setStart(bool startIn) {
	pthread_mutex_lock(&_dataMutex);
	_data.start = startIn;
    // if (_data.start && _data.manual) {
    //     _data.manual = false;
    // }
	pthread_mutex_unlock(&_dataMutex);
}

bool GlobalState::getStart() {
	// pthread_mutex_lock(&_dataMutex);
	bool ret = _data.start;
	// pthread_mutex_unlock(&_dataMutex);
	return ret;
}
/*
void GlobalState::setManual(bool startIn) {
    pthread_mutex_lock(&_dataMutex);
    _data.manual = startIn;
    if (_data.start && _data.manual) {
        _data.start = false;
    }
    pthread_mutex_unlock(&_dataMutex);
}

bool GlobalState::getManual() {
    pthread_mutex_lock(&_dataMutex);
    bool ret = _data.manual;
    pthread_mutex_unlock(&_dataMutex);
    return ret;
}

std::vector<std::array<float, 2>> getGreenBlob() {
    pthread_mutex_lock(&_dataMutex);
    return greenBlobs;
    pthread_mutex_unlock(&_dataMutex);
}

*/