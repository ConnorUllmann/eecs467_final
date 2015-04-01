#ifndef GAME_PLAYER_HPP
#define GAME_PLAYER_HPP

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
// #include <math/point.hpp>
#include "Board.hpp"

#include <lcm/lcm-cpp.hpp>
#include "ColorRecognizer.hpp"
#include "lcmtypes/ttt_turn_t.hpp"
#include "LcmHandler.hpp"
#include "BlobDetector.hpp"

// typedef eecs467::Point<int> IntPoint;

class GamePlayer {
private:
	pthread_t _gamePlayerPid;

    OBJECT _ballColor;


    int _direction;

    // pthread_t _messagesPid;
    // OBJECT _ourColor;
    // OBJECT _theirColor;

    // int32_t their_turn;
    // int32_t our_turn;

    // bool go;
    static GamePlayer* _instance;

    mutable pthread_mutex_t _GamePlayerMutex;
    GamePlayer();

    int calculateBallDirection(BlobDetector::Blob& b1, BlobDetector::Blob& b2);

public:
    // Board _board;
    std::vector<BlobDetector::Blob> _ballPos;

	void init(OBJECT ballColor);

	static GamePlayer* instance();

	// void setBoard(const Board& board);

	// void checkIfYourTurn(const ttt_turn_t* msg);

	void launchThreads();
    
    std::vector<BlobDetector::Blob> getBallPos();

    int getDirection();

    void clearBallPos();

private:
	static void* gameThread(void* args);

	// static void* publishMessages(void* args);
};

#endif /* GAME_PLAYER_HPP */
