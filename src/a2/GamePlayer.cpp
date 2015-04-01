#include "GamePlayer.hpp"
#include "BlobDetector.hpp"
#include "GlobalState.hpp"
#include "Constants.hpp"
#include "CalibrationHandler.hpp"
#include "CoordinateConverter.hpp"
#include "Arm.hpp"
#include <iostream>
#include <vector>

GamePlayer* GamePlayer::_instance = new GamePlayer;

GamePlayer::GamePlayer() {
	_ballColor = NONE;
    _ballPos.clear();
    _direction = STATION;
}

GamePlayer* GamePlayer::instance() {
	return _instance;
}

void GamePlayer::init(OBJECT ballColor)
{
	_ballColor = ballColor;
    _ballPos.clear();
    _direction = STATION;
}


void GamePlayer::launchThreads() {
	pthread_create(&_gamePlayerPid, NULL,
		&GamePlayer::gameThread, this);
	// pthread_create(&_messagesPid, NULL,
	// 	&GamePlayer::publishMessages, this);
				
}
/*
void* GamePlayer::publishMessages(void* args){
	GamePlayer* state = (GamePlayer*) args;
	ttt_turn_t turnMessage;

	while(1) {
		pthread_mutex_lock(&state->_GamePlayerMutex);
		turnMessage.utime = 0;
		turnMessage.turn = state->our_turn;
		if(state->_ourColor == GREENBALL)
			LcmHandler::instance()->getLcm()->publish("GREEN_TURN", &turnMessage);
		else
			LcmHandler::instance()->getLcm()->publish("RED_TURN", &turnMessage);
		pthread_mutex_unlock(&state->_GamePlayerMutex);
		usleep(5e4);
	}
	return NULL;
}
*/
void* GamePlayer::gameThread(void* args) {
	GamePlayer* state = (GamePlayer*) args;

    int newDir = LEFT;

    while (1) {
        while (!GlobalState::instance()->getStart()) {usleep(10000);};

		pthread_mutex_lock(&state->_GamePlayerMutex);

		RenderInfo renderInfo(GlobalState::instance()->getData());


        if (renderInfo.im == nullptr) {
            pthread_mutex_unlock(&state->_GamePlayerMutex);
			continue;
        }
		// take in board state and blobs or w/e
		std::vector<BlobDetector::Blob> blobs = 
			BlobDetector::findGreenBlobs(renderInfo.im,
			CalibrationHandler::instance()->getCalibration(),
			blobMinPixels);

        if (blobs.size() <= 0) {
            std::cout << "Can't detect any green ball\n";
            pthread_mutex_unlock(&state->_GamePlayerMutex);
            usleep(100000);
            continue;
        }

        if (blobs.size() > 1) {
            std::cout << "Detected multiple ball\n";
        }

// std::cout << "blob size " << blobs.size() << "\n";
		
        if (!state->_ballPos.empty()) {
            newDir = state->calculateBallDirection(state->_ballPos.back(), blobs[blobs.size()/2]);

            if (state->_ballPos.size() >= 50000) {
                state->_ballPos.clear();
            }
        }


        if (newDir != STATION) {

// std::cout << "new dir " << newDir << "\n";

            if ((state->_direction == LEFT && newDir == RIGHT) ||
                    (state->_direction == RIGHT && newDir == LEFT) ) {
                state->_ballPos.clear();
            }
            state->_direction = newDir;

            state->_ballPos.push_back(blobs[blobs.size()/2]);
// std::cout << "ballpos size " << state->_ballPos.size() << " x " << blobs[blobs.size()/2].x << " y " << blobs[blobs.size()/2].y << " \n";
        }


		pthread_mutex_unlock(&state->_GamePlayerMutex);
        
        // ################prediction here

        // ################ai here

		while (1) {
			if (!Arm::instance()->inMotion()) {
				break;
			}
            usleep(1000);
		}

	}
	pthread_mutex_unlock(&state->_GamePlayerMutex);

	return NULL;
}

/*
void GamePlayer::checkIfYourTurn(const ttt_turn_t* msg){
	pthread_mutex_lock(&_GamePlayerMutex);
	their_turn = msg->turn;
	pthread_mutex_unlock(&_GamePlayerMutex);
}
*/

int GamePlayer::calculateBallDirection(BlobDetector::Blob& b1, BlobDetector::Blob& b2) {
    int dir = b1.x-b2.x;

// std::cout << "b1 " << b1.x << "," << b1.y << " | b2 " << b2.x << "," << b2.y << endl;

    if (abs(dir) <= 1 && abs(b1.y-b2.y) <= 1) {
        return STATION;
    }
    if (dir > 0) {
        return LEFT;
    }
    if (dir < 0) {
        return RIGHT;
    }
    return STATION;
}

std::vector<BlobDetector::Blob> GamePlayer::getBallPos() {
    // pthread_mutex_lock(&_GamePlayerMutex);
    
    std::vector<BlobDetector::Blob> ret = _ballPos;
    
    // pthread_mutex_unlock(&_GamePlayerMutex);
    return ret;
}

int GamePlayer::getDirection() {
    // pthread_mutex_lock(&_GamePlayerMutex);
    
    int ret = _direction;
    
    // pthread_mutex_unlock(&_GamePlayerMutex);
    return ret;
}

void GamePlayer::clearBallPos() {
    _ballPos.clear();
}