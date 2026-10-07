#ifndef GRID_H
#define GRID_H

#include "../constants.h"

void displayBlocksAndGoals(int [DIMENSIONS][DIMENSIONS]);
void drawGrid(void);
void initGrid(int [DIMENSIONS][DIMENSIONS]);
void addObstacle(int [DIMENSIONS][DIMENSIONS], int, int);
void addMarker(int [DIMENSIONS][DIMENSIONS], int, int);
void addHome(int [DIMENSIONS][DIMENSIONS], int, int);

#endif
