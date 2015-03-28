#include <iostream>
#include <string>
#include <stdlib.h>
#include "Board.hpp"

using namespace std;

Board::Board() : turnState(0)
{
	reset();
}

bool Board::indexInBounds(int x, int y) {
	return (x >= 0 && x < 3 && y >= 0 && y < 3);
}

void Board::printBoard()
{
	cout << "---" << endl;
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			cout << toChar(i, j);
		}
		cout << endl;
	}
	cout << "---" << endl;
} 

int Board::getNextMove(int player) //player == 1 means player 1. player == 2 means player 2.
{
	//If I can win, move there.
	int wm = hasWinningMove(player);
	if(wm != -1)
	{
		cout << "Winning move for: " << toChar(player) << endl;
		return wm;
	}
	 
	//If my opponent can win, block it.
	wm = hasWinningMove(toOpponent(player));
	if(wm != -1)
	{
		cout << "Blocking " << toChar(toOpponent(player)) << "'s winning move." << endl;
		return wm;
	}

	//If I can set up a double fork, do it.
	wm = hasDoubleForkMove(player);
	if(wm != -1)
	{
		cout << "Double forking move for: " << toChar(player) << endl;
		return wm;
	}

	//See if I have a fork to play.
	wm = hasForkingMove(player);
	if(wm != -1)
	{
		cout << "Forking move for: " << toChar(player) << endl;
		return wm;
	}

	//See if the opponent has a fork to play and block it.
	wm = hasForkingMove(toOpponent(player));
	if(wm != -1)
	{
		cout << "Blocking " << toChar(toOpponent(player)) << "'s forking move." << endl;
		return wm;
	}

	//If they can set up a double fork, don't move to that position.
	int totalMoves = 0;
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			if(board[i][j] == player || board[i][j] == toOpponent(player))
				totalMoves++;
		}
	}
	//Opponent is set up for a double fork.
	bool b=totalMoves == 1 &&
		  (board[0][0] == toOpponent(player) ||
		   board[2][0] == toOpponent(player) ||
		   board[0][2] == toOpponent(player) ||
		   board[2][2] == toOpponent(player));

	//Take middle if it's open and won't lead to a double fork.
	if(board[1][1] == 0 && !b) {
		return toInd(1, 1);
	}
	 
	//If a corner is open, pick a random corner.
	while(openCorner())
	{
		int i = (rand() % 2) * 2;
		int j = (rand() % 2) * 2;
		if(board[i][j] == 0) {
			return toInd(i, j);
		}
	}
	 
	//Choose a random unclaimed position.
	while(!full())
	{
		int i = rand() % 3;
		int j = rand() % 3;
		if(board[i][j] == 0)
			return toInd(i, j);
	}
	return -1; //Can't move anywhere.               
}

void Board::clearBoard() {
	for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				board[i][j] = 0;
}


int& Board::operator()(int x, int y) {
	return board[x][y];
}

//Returns -1 if there is no winning move. Returns the index (0-8, rows by columns)
//of the position that would win. p is the player (1 or 2).
int Board::hasWinningMove(int p)
{
	//Check horizontals
	for(int i = 0; i < 3; i++)
	{
		int ind = -1;
		int a = 0;
		for(int j = 0; j < 3; j++)
		{
			if(board[i][j] == p)
				a++;
			else if(board[i][j] == toOpponent(p))
				a = -1000;
			else
				ind = toInd(i, j);
		}
		if(a == 2)
			return ind;
	}

	//Check verticals
	for(int i = 0; i < 3; i++)
	{
		int ind = -1;
		int b = 0;
		for(int j = 0; j < 3; j++)
		{
			if(board[j][i] == p)
				b++;
			else if(board[j][i] == toOpponent(p))
				b = -1000;
			else
				ind = toInd(j, i);
		}
		if(b == 2)
			return ind;
	}

	//Check left-diagonal
	int c = 0;
	int ind = -1;
	for(int i = 0; i < 3; i++)
	{
		if(board[i][i] == p)
			c++;
		else if(board[i][i] == toOpponent(p))
			c = -1000;
		else
			ind = toInd(i, i);
	}
	if(c == 2)
		return ind;

	//Check right-diagonal
	int d = 0;
	ind = -1;
	for(int i = 0; i < 3; i++)
	{
		if(board[2-i][i] == p)
			d++;
		else if(board[2-i][i] == toOpponent(p))
			d = -1000;
		else
			ind = toInd(2-i, i);
	}
	if(d == 2)
		return ind;

	return -1;
}

//Returns the position player should move to in order to make a fork. -1 if none.
int Board::hasForkingMove(int p)
{
	int maxInd = -1;
	int maxChecks = 0;
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			if(board[i][j] != 0)
				continue;

			int n = numChecksAt(i, j, p);
			if(n > maxChecks)
			{
				maxChecks = n;
				maxInd = toInd(i, j);
			}
		}
	}
	if(maxChecks >= 2)
		return maxInd;
	return -1;
}


int Board::hasDoubleForkMove(int p)
{
	int totalMoves = 0;
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			if(board[i][j] == p || board[i][j] == toOpponent(p))
				totalMoves++;
		}
	}

	if(totalMoves == 2)
	{
		if(board[1][1] == toOpponent(p))
		{
			if(board[0][0] == p)
				return toInd(2, 2);
			if(board[2][0] == p)
				return toInd(0, 2);
			if(board[0][2] == p)
				return toInd(2, 0);
			if(board[2][2] == p)
				return toInd(0, 0);
		}
	}
	return -1;
}

//How many ways the opponent is put in "check" (ala chess) by a move by player p at position <row, col>.
int Board::numChecksAt(int row, int col, int p)
{
	int a = 0;
	int b = 0;
	int c = 0;
	int d = 0;
	for(int i = 0; i < 3; i++)
	{
		if(board[row][i] == p)
			a++;
		else if(board[row][i] == toOpponent(p))
			a = -1000;

		if(board[i][col] == p)
			b++;
		else if(board[i][col] == toOpponent(p))
			b = -1000;
		 
		if(isOnLeftDiag(row, col))
		{
			if(board[i][i] == p)
				c++;
			else if(board[i][i] == toOpponent(p))
				c = -1000;
		}

		if(isOnRightDiag(row, col))
		{
			if(board[2-i][i] == p)
				d++;
			else if(board[2-i][i] == toOpponent(p))
				d = -1000;
		}
	}
	//Add up the number of times we put the opponent in "check" by the number of
	//rows/cols/diags who have one of ours and two empties.
	return (a == 1 ? 1 : 0) + (b == 1 ? 1 : 0) + 
				(c == 1 ? 1 : 0) + (d == 1 ? 1 : 0);
}

void Board::reset()
{
	turnState = 0;
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			board[i][j] = 0;
}

bool Board::hasWon(int p)
{
	for(int i = 0; i < 3; i++)
	{
		//Check horizontals.
		if(board[i][0] == p &&
		 board[i][1] == p &&
		 board[i][2] == p)
			return true;

		//Check verticals.
		if(board[0][i] == p &&
		 board[1][i] == p &&
		 board[2][i] == p)
			return true;
	}

	if(board[0][0] == p &&
		board[1][1] == p &&
		board[2][2] == p)
		return true;

	if(board[2][0] == p &&
		board[1][1] == p &&
		board[0][2] == p)
		return true;

	return false;
}

bool Board::openCorner() { return board[0][0] == 0 || board[2][0] == 0 || board[0][2] == 0 || board[2][2] == 0; }

bool Board::full() { for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) if(board[i][j] == 0) return false; return true; }

bool Board::isOnLeftDiag( int r, int c) { return (r == 0 && c == 0) || (r == 1 && c == 1) || (r == 2 && c == 2); }

bool Board::isOnRightDiag(int r, int c) { return (r == 0 && c == 2) || (r == 1 && c == 1) || (r == 2 && c == 0); }

int Board::toOpponent(int p) { if(p != 2) return 2; return 1; }

int Board::toInd(int r, int c) { return r * 3 + c; }

int Board::toRow(int i) { return i / 3; }

int Board::toCol(int i) { return i % 3; }

char Board::toChar(int p) { return (p == 1 ? 'X' : (p == 2 ? 'O' : ' ')); };

char Board::toChar(int r, int c) { return toChar(board[r][c]); };
/*
int main()
{
	Board b;
	
	while(1)
	{
		int p = 0;
		if(b.turnState == 0 || b.turnState == 1)
			p = 1;
		else
			p = 2;
			
		int m = b.getNextMove(p);
		if(m == -1)
		{
			cout << "IT'S BROKEN!" << endl;
			cin.get();
			b.reset();
		}
		else
		{
			b.board[b.toRow(m)][b.toCol(m)] = p;
			b.turnState = b.toOpponent(b.turnState);
		}
		
		b.printBoard();
		if(b.hasWon(1))
		{
			cout << "PLAYER " << b.toChar(1) << " HAS WON!" << endl;
			b.reset();
		}
		if(b.hasWon(2))
		{
			cout << "PLAYER " << b.toChar(2) << " HAS WON!" << endl;
			b.reset();
			cin.get();
		}
		if(b.full())
		{
			cout << "NO MORE MOVES." << endl;
			b.reset();
		}
	}
	
	system("pause");
	return 0;
}
*/
