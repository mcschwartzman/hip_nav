#pragma once

#include <vector>

// creating a map means creating a bunch of GridCells
// 0. You start off without a map, robot must create one from env info (sensors or preloading)
// 1. A given cell must be created WITH an X and a Y

class GridCell {

private:
	int x;
	int y;



public:

	std::vector<GridCell*> neighbors;
	GridCell* parent;
	
	bool visited = false;


	GridCell(int a, int b) {
		x = a;
		y = b;
	}

	void setX(int a) {
		x = a;
	}

	void setY(int a) {
		y = a;
	}

	int getX() {
		return x;
	}

	int getY() {
		return y;
	}
};