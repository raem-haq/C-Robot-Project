#include "../drawing/graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "grid.h"
#include "robot.h"
#include "../constants.h"

static const int STEPS = 1;

static void moveOneCell(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS],
                        Pixel xDirection, Pixel yDirection) {
    Pixel increment = SQUARE_SIDE_LENGTH / STEPS;

    for (int i = 0; i < STEPS; i++) {
        clear();
        robotPtr->xP += xDirection * increment;
        robotPtr->yP += yDirection * increment;
        drawForeground(robotPtr, map);
        sleep(WAIT_TIME_MOVE);
    }
}

int atMarker(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int x = robotPtr->xP / SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP / SQUARE_SIDE_LENGTH;
    return map[x][y] >= 3;
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
    robotPtr->isCarryingMarker = false;
}

void right(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    clear();
    robotPtr->direction = (Direction)((robotPtr->direction + 1) % 4);
    drawForeground(robotPtr, map);
    sleep(WAIT_TIME_TURN);
}

void forward(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    switch (robotPtr->direction) {
    case DIRECTION_NORTH:
        moveOneCell(robotPtr, map, 0, -1);
        break;
    case DIRECTION_EAST:
        moveOneCell(robotPtr, map, 1, 0);
        break;
    case DIRECTION_SOUTH:
        moveOneCell(robotPtr, map, 0, 1);
        break;
    case DIRECTION_WEST:
        moveOneCell(robotPtr, map, -1, 0);
        break;
    }
}

void drawRobot(Robot *robotPtr) {
    if (robotPtr->isCarryingMarker) {
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
