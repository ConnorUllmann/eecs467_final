#ifndef GLOBAL_STATE_HPP
#define GLOBAL_STATE_HPP


#include <vector>
#include <array>
#include <pthread.h>
#include "imagesource/image_u32.h"

struct RenderInfo {
	image_u32_t* im;
	std::vector<std::array<float, 2>> redBlobs;
	std::vector<std::array<float, 2>> greenBlobs;
	std::vector<std::array<float, 2>> blueBlobs;
	bool start;

	RenderInfo();
	~RenderInfo();
	void copy(const RenderInfo& info);
	void operator=(const RenderInfo& info);
};


class GlobalState {
private:
	pthread_mutex_t _dataMutex;
	RenderInfo _data;
	GlobalState();
	static GlobalState* _instance;

public:
	static GlobalState* instance();

	void setData(const RenderInfo& data);

	RenderInfo getData();

	void setStart(bool startIn);

	bool getStart();
};


#endif /* GLOBAL_STATE_HPP */
