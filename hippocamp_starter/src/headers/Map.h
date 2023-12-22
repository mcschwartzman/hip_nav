#pragma once
#include <vector>
#include <cmath>

#include "GridCell.h"

// creating a map means creating a bunch of GridCells
// 0. You start off without a map, robot must create one from env info (sensors or preloading)
// 1. A given cell must be created WITH an X and a Y

class Map {

private:
	
	int size;


public:
	
	std::vector<GridCell> nodeList;

	Map(int s) {
		size = s;

		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				GridCell g(i, j);
				nodeList.push_back(g);
			}
		}

	}

	void mapCell(int x, int y) {
		GridCell g(x, y);
		nodeList.push_back(g);
	}

	GridCell getCell(int x, int y) {

		int roundedX = round(x);
		int roundedY = round(y);

		for (int i = 0; i < nodeList.size(); i++){

			if (nodeList[i].getX() == roundedX){
				if (nodeList[i].getY() == roundedY){
					return nodeList[i];
				}
			}
		}
	}

};