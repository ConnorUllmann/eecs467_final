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
	_ourColor = NONE;
	_theirColor = NONE;
	their_turn = 0;
	our_turn = 0;
	go = false;
}

GamePlayer* GamePlayer::instance() {
	return _instance;
}

void GamePlayer::init(OBJECT ourColor)
{
	_ourColor = ourColor;
	if (_ourColor == REDBALL) {
		_theirColor = GREENBALL;
	} else {
		_theirColor = REDBALL;
	}
	go = false;

	our_turn = 0;
	their_turn = 0;
}

void GamePlayer::launchThreads() {
	pthread_create(&_gamePlayerPid, NULL,
		&GamePlayer::gameThread, this);
	pthread_create(&_messagesPid, NULL,
		&GamePlayer::publishMessages, this);
				
}

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

void* GamePlayer::gameThread(void* args) {
	GamePlayer* state = (GamePlayer*) args;

	while (!GlobalState::instance()->getStart());

	while (1) {
		pthread_mutex_lock(&state->_GamePlayerMutex);

		if (state->_ourColor == REDBALL) {
			if (state->their_turn != state->our_turn) {
				pthread_mutex_unlock(&state->_GamePlayerMutex);
				continue;
			}
		} else if (state->_ourColor == GREENBALL) {
			if (state->their_turn <= state->our_turn) {
				pthread_mutex_unlock(&state->_GamePlayerMutex);
				continue;
			}
		} else {
			pthread_mutex_unlock(&state->_GamePlayerMutex);
			continue;
		}

		printf("Turn: %d\n", state->our_turn);
		state->go = false;
    
		RenderInfo renderInfo = GlobalState::instance()->getData();


        if (renderInfo.im == nullptr) {
            pthread_mutex_unlock(&state->_GamePlayerMutex);
			continue;
        }
		// take in board state and blobs or w/e
		std::vector<BlobDetector::Blob> blobs = 
			BlobDetector::findBlobs(renderInfo.im,
			CalibrationHandler::instance()->getCalibration(),
			blobMinPixels);

		// separate out blobs
		std::vector<BlobDetector::Blob> unusedBalls;
		state->_board.clearBoard();
		for (auto& blob : blobs) {
			if (blob.type == NONE) {
				pthread_mutex_unlock(&state->_GamePlayerMutex);
				continue;
			}
			std::array<float, 2> globalCoords = 
				CoordinateConverter::imageToGlobal(std::array<int, 2>{{blob.x, blob.y}});
			std::array<int, 2> boardCoords = 
				CoordinateConverter::globalToBoard(globalCoords);

			if (blob.type == state->_ourColor) {
				if (Board::indexInBounds(boardCoords[0], boardCoords[1])) {
					state->_board(boardCoords[0], boardCoords[1]) = state->_ourColor;
				} else {
					unusedBalls.push_back(blob);
				}
			} else {
				if (Board::indexInBounds(boardCoords[0], boardCoords[1])) {
					state->_board(boardCoords[0], boardCoords[1]) = state->_theirColor;
				}
			}
		}

		state->_board.printBoard();
		int move = state->_board.getNextMove(state->_ourColor);

		if (move == -1 || state->_board.full()) {
			printf("End Game!\n");
			break;
		}
		if(state->_board.hasWon(REDBALL))
		{
			printf("Red player has won!\n");
			break;
		}
		if(state->_board.hasWon(GREENBALL))
		{
			printf("Green player has won!\n");
			break;
		}
		std::array<int, 2> boardMove{{Board::toRow(move),
			Board::toCol(move)}};
		//printf("%d, %dcurrent\n", boardMove[0], boardMove[1]);

		// pick up random ball
		printf("Number of unused balls: %d\n", unusedBalls.size());
		if (!unusedBalls.empty()) {
			BlobDetector::Blob grabBall = unusedBalls.front();
			std::array<float, 2> globalCoords = 
				CoordinateConverter::imageToGlobal(std::array<int, 2>{{grabBall.x, grabBall.y}});
			printf("global arm: %f, %f\n", globalCoords[0], globalCoords[1]);
			if (!Arm::instance()->addCommandGrabBall(globalCoords)) {
				printf("Can't reach ball\n");
			}
		}
		// put ball
		Arm::instance()->addCommandDropBall(boardMove);

		// state->our_turn++;
		pthread_mutex_unlock(&state->_GamePlayerMutex);

		while (1) {
			if (!Arm::instance()->inMotion()) {
				break;
			}
		}

		pthread_mutex_lock(&state->_GamePlayerMutex);
		state->our_turn++;
		pthread_mutex_unlock(&state->_GamePlayerMutex);
		printf("\n");
	}
	pthread_mutex_unlock(&state->_GamePlayerMutex);

	return NULL;
}

void GamePlayer::checkIfYourTurn(const ttt_turn_t* msg){
	pthread_mutex_lock(&_GamePlayerMutex);
	their_turn = msg->turn;
	pthread_mutex_unlock(&_GamePlayerMutex);
}

