#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

class Board
{
private:
	int turnState;

public:
	int board[3][3];
	
	Board();

	static bool indexInBounds(int x, int y);
	void printBoard();
	int getNextMove(int player);
	static int toRow(int i);
	static int toCol(int i);
	bool hasWon(int p);
	bool full();
	void clearBoard();
	int& operator()(int x, int y);

private:
	int hasWinningMove(int p);
	int hasForkingMove(int p);
	int hasDoubleForkMove(int p);
	int numChecksAt(int row, int col, int p);
	void reset();
	bool openCorner();
	bool isOnLeftDiag( int r, int c);
	bool isOnRightDiag(int r, int c);
	int toOpponent(int p);
	int toInd(int r, int c);
	char toChar(int p);
	char toChar(int r, int c);
};
