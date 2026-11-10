#include "../drawing/graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "grid.h"
#include "robot.h"
#include "../constants.h"

void moveEast(Robot *, int [DIMENSIONS][DIMENSIONS]);
void moveWest(Robot *, int [DIMENSIONS][DIMENSIONS]);
void moveUp(Robot *, int [DIMENSIONS][DIMENSIONS]);
void moveDown(Robot *, int [DIMENSIONS][DIMENSIONS]);


int atMarker(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {

    int xC = (robotPtr->xP) / SQUARE_SIDE_LENGTH;//convert from pixels to coordinates
    int yC = (robotPtr->yP) / SQUARE_SIDE_LENGTH;
    return (map[xC][yC] >= 3);
}

void drawForeground(Robot* robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    displayBlocksAndGoals(map);
    drawRobot(robotPtr);
}

int canForward(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int result;

    int xC = (robotPtr->xP) / SQUARE_SIDE_LENGTH;//again convert from pixels to coordinates
    int yC = (robotPtr->yP) / SQUARE_SIDE_LENGTH;

    switch (robotPtr->direction) {
    case DIRECTION_NORTH:
        result = ((yC - 1) >= 0);//check if you would be in the grid
        if (result) {
            //then check if you would be on an obstacle
            result = (map[xC][yC - 1] != 0);
        }
        break;
    case DIRECTION_EAST:
        result = ((xC + 1) < DIMENSIONS);
        if (result) {
            result = (map[xC + 1][yC] != 0);
        }
        break;
    case DIRECTION_SOUTH:
        result = ((yC + 1) < DIMENSIONS);
        if (result) {
            result = (map[xC][yC + 1] != 0);
        }
        break;
    case DIRECTION_WEST:
        result = ((xC - 1) >= 0);
        if (result) {
            result = (map[xC - 1][yC] != 0);
        }
        break;
    default:
        result = 0;
        break;
    }
    return result;
}



void left(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    clear();
    robotPtr->direction = (Direction)((robotPtr->direction + 3) % 4);
    drawForeground(robotPtr, map);
    sleep(WAIT_TIME_TURN);
}

void initRobot(Robot *robotPtr, Pixel initXP, Pixel initYP, Direction initDirection) {
    robotPtr->xP = initXP;
    robotPtr->yP = initYP;
    robotPtr->direction = initDirection;
    robotPtr->isCarryingMarker = 0;
}

void right(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    clear();
    robotPtr->direction = (Direction)((robotPtr->direction + 1) % 4);
    drawForeground(robotPtr, map);
    sleep(WAIT_TIME_TURN);
}

void moveUp(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int N = 5;
    Pixel inc = SQUARE_SIDE_LENGTH / N;
    for (int i = 0; i < N; i++) {
        clear();
        robotPtr->yP -= inc;
        drawForeground(robotPtr, map);
        sleep(WAIT_TIME_MOVE);
    }
}

void moveDown(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int N = 5;
    Pixel inc = SQUARE_SIDE_LENGTH / N;
    for (int i = 0; i < N; i++) {
        clear();
        robotPtr->yP += inc;
        drawForeground(robotPtr, map);
        sleep(WAIT_TIME_MOVE);
    }
}

void forward(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    switch (robotPtr->direction) {
    case DIRECTION_NORTH:
        moveUp(robotPtr, map);
        break;
    case DIRECTION_EAST:
        moveEast(robotPtr, map);
        break;
    case DIRECTION_SOUTH:
        moveDown(robotPtr, map);
        break;
    case DIRECTION_WEST:
        moveWest(robotPtr, map);
        break;
    }
}

void drawRobot(Robot *robotPtr) {
    if (robotPtr->isCarryingMarker == 1) {
        setColour(yellow);
    } else {
        setColour(gray);
    }

    fillOval(robotPtr->xP, robotPtr->yP, SQUARE_SIDE_LENGTH, SQUARE_SIDE_LENGTH);
    
    // eyes
    setColour(black);
    if (robotPtr->direction == DIRECTION_NORTH) {
        fillRect(robotPtr->xP + SQUARE_SIDE_LENGTH / 3 - SQUARE_SIDE_LENGTH / 5, robotPtr->yP + SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
        fillRect(robotPtr->xP + 2 * SQUARE_SIDE_LENGTH / 3, robotPtr->yP + SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
    } else if (robotPtr->direction == DIRECTION_EAST) {
        fillRect(robotPtr->xP + SQUARE_SIDE_LENGTH / 3, robotPtr->yP + SQUARE_SIDE_LENGTH / 3, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
        fillRect(robotPtr->xP + 2 * SQUARE_SIDE_LENGTH / 3, robotPtr->yP + SQUARE_SIDE_LENGTH / 3, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
    } else if (robotPtr->direction == DIRECTION_SOUTH) {
        fillRect(robotPtr->xP + SQUARE_SIDE_LENGTH / 3, robotPtr->yP + 2 * SQUARE_SIDE_LENGTH / 3, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
        fillRect(robotPtr->xP + 2 * SQUARE_SIDE_LENGTH / 3, robotPtr->yP + 2 * SQUARE_SIDE_LENGTH / 3, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
    } else if (robotPtr->direction == DIRECTION_WEST) {
        fillRect(robotPtr->xP, robotPtr->yP + SQUARE_SIDE_LENGTH / 3, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
        fillRect(robotPtr->xP + SQUARE_SIDE_LENGTH / 3, robotPtr->yP + SQUARE_SIDE_LENGTH / 3, SQUARE_SIDE_LENGTH / 5, SQUARE_SIDE_LENGTH / 5);
    }
    
}


void goHome(Robot *robotPtr, char* moveStack, int noOfMoves, int map[DIMENSIONS][DIMENSIONS]) {
    char instruct;
    turnAround(robotPtr, map);
    for (int i = noOfMoves - 1; i >= 0; i--) {
        instruct = *(moveStack+i);
        switch (instruct) {
            case 'F':
                forward(robotPtr, map);
                break;
            case 'L':
                right(robotPtr, map);//need to invert rotation (right/left) but not forward 
                break;
            case 'R':
                left(robotPtr, map);
                break;
            default:
                break;
        }
    }
}

void turnAround(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    //two rights or lefts make you turn around
    right(robotPtr, map);
    right(robotPtr, map);
}

void moveEast(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int N = 5;
    Pixel inc = SQUARE_SIDE_LENGTH / N;
    for (int i = 0; i < N; i++) {
        clear();
        robotPtr->xP += inc;
        drawForeground(robotPtr, map);
        sleep(WAIT_TIME_MOVE);
    }
}

void moveWest(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int N = 5;
    Pixel inc = SQUARE_SIDE_LENGTH / N;
    for (int i = 0; i < N; i++) {
        clear();
        robotPtr->xP -= inc;
        drawForeground(robotPtr, map);
        sleep(WAIT_TIME_MOVE);
    }
}