#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>

#include "../constants.h"

typedef int Pixel;

typedef enum { DIRECTION_NORTH, DIRECTION_EAST, DIRECTION_SOUTH, DIRECTION_WEST } Direction;

typedef struct {
    Pixel xP;
    Pixel yP;
    Direction direction;
    bool isCarryingMarker;
} Robot;

void drawRobot(Robot *);

int canForward(Robot *, int[DIMENSIONS][DIMENSIONS]);
void left(Robot *, int[DIMENSIONS][DIMENSIONS]);
void right(Robot *, int[DIMENSIONS][DIMENSIONS]);
void forward(Robot *, int[DIMENSIONS][DIMENSIONS]);
void goHome(Robot *, char *, int, int[DIMENSIONS][DIMENSIONS]);
void turnAround(Robot *, int[DIMENSIONS][DIMENSIONS]);

int atMarker(Robot *, int[DIMENSIONS][DIMENSIONS]);
void initRobot(Robot *, Pixel, Pixel, Direction);
void drawForeground(Robot *, int[DIMENSIONS][DIMENSIONS]);

#endif