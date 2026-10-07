#ifndef ROBOT_H
#define ROBOT_H

#include "../constants.h"

typedef int Pixel;

typedef struct {
    Pixel xP;
    Pixel yP;
    int direction;
    int isCarryingMarker;
} Robot;

void drawRobot(Robot *);

int canForward(Robot *, int [DIMENSIONS][DIMENSIONS]);
void left(Robot *, int [DIMENSIONS][DIMENSIONS]);
void right(Robot *, int [DIMENSIONS][DIMENSIONS]);
void forward(Robot *, int [DIMENSIONS][DIMENSIONS]);
void goHome(Robot *, char*, int, int [DIMENSIONS][DIMENSIONS]);
void turnAround(Robot *, int [DIMENSIONS][DIMENSIONS]);

int atMarker(Robot *, int [DIMENSIONS][DIMENSIONS]);
void initRobot(Robot *, Pixel, Pixel, int);
void drawForeground(Robot*, int [DIMENSIONS][DIMENSIONS]);

#endif