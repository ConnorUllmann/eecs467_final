#ifndef GAME_PLAYER_HPP
#define GAME_PLAYER_HPP

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include "Board.hpp"

#include <lcm/lcm-cpp.hpp>
#include "ColorRecognizer.hpp"
#include "lcmtypes/ttt_turn_t.hpp"
#include "LcmHandler.hpp"

class GamePlayer {
private:
	pthread_t _gamePlayerPid;
	pthread_t _messagesPid;
	OBJECT _ourColor;
	OBJECT _theirColor;

	int32_t their_turn;
	int32_t our_turn;

	bool go;
	static GamePlayer* _instance;

	mutable pthread_mutex_t _GamePlayerMutex;
	GamePlayer();

public:
	Board _board;

	void init(OBJECT ourColor);

	static GamePlayer* instance();

	void setBoard(const Board& board);

	void checkIfYourTurn(const ttt_turn_t* msg);

	void launchThreads();

private:
	static void* gameThread(void* args);

	static void* publishMessages(void* args);
};

#endif /* GAME_PLAYER_HPP */
