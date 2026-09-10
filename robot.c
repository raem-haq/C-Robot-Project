#include "graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "grid.h"
#include "constants.h"

typedef struct
{
    int xP;
    int yP;
    int direction;
    int isCarryingMarker;
} Robot;

void drawRobot(Robot *);

int canForward(Robot *, int **);
void left(Robot *, int**);
void right(Robot *, int**);

void moveEast(Robot *, int**);
void moveWest(Robot *, int**);
void moveUp(Robot *, int**);
void moveDown(Robot *, int**);
void forward(Robot *, int**);

void turnAround(Robot *, int**);
void goHome(Robot *, char*, int, int**);

int atMarker(Robot *, int **);
void initRobot(Robot *, int, int, int);

void drawForeground(Robot*, int**);


int atMarker(Robot *robotPtr, int **map)
{

    int xC = (robotPtr->xP) / squareSideLength;//convert from pixels to coordinates
    int yC = (robotPtr->yP) / squareSideLength;
    return ((*(*(map + xC) + yC)) >= 3);
}

void drawForeground(Robot* robotPtr, int** map)
{
    displayBlocksAndGoals(map);
    drawRobot(robotPtr);
}

int canForward(Robot *robotPtr, int **map)
{
    int result;

    int xC = (robotPtr->xP) / squareSideLength;//again convert from pixels to coordinates
    int yC = (robotPtr->yP) / squareSideLength;

    switch (robotPtr->direction)
    {
    case 0:
        result = ((yC - 1) >= 0);//check if you would be in the grid
        if (result)
        {
            //then check if you would be on an obstacle
            result = result * (*(*(map + xC) + yC - 1) != 0);
        }
        break;
    case 1:
        result = ((xC + 1) < dimensions);
        if (result)
        {
            result = result && (*(*(map + xC + 1) + yC) != 0);
        }
        break;
    case 2:
        result = ((yC + 1) < dimensions);
        if (result)
        {
            result = result * (*(*(map + xC) + yC + 1) != 0);
        }
        break;
    case 3:
        result = ((xC - 1) >= 0);
        if (result)
        {
            result = result * (*(*(map + xC - 1) + yC) != 0);
        }
        break;
    default:
        result = 0;
        break;
    }
    return result;
}



void left(Robot *robotPtr, int** map)
{   
    clear();
    robotPtr->direction = (robotPtr->direction + 3) % 4;//this will decrement the direction with wrap-around
    drawForeground(robotPtr, map);
    sleep(waitTimeTurn);
}

void initRobot(Robot *robotPtr, int initXP, int initYP, int initD)
{
    robotPtr->xP = initXP;
    robotPtr->yP = initYP;
    robotPtr->direction = initD;
    robotPtr->isCarryingMarker = 0;
}

void right(Robot *robotPtr, int** map)
{
    clear();
    robotPtr->direction = (robotPtr->direction + 1) % 4;//increment the direction with wrap-around
    drawForeground(robotPtr, map);
    sleep(waitTimeTurn);
}

void moveUp(Robot *robotPtr, int** map)
{
    int N = 5;
    int inc = squareSideLength / N;
    for (int i = 0; i < N; i++)
    {
        clear();
        robotPtr->yP -= inc;
        drawForeground(robotPtr, map);
        sleep(waitTimeMove);
    }
}

void moveDown(Robot *robotPtr, int** map)
{
    int N = 5;
    int inc = squareSideLength / N;
    for (int i = 0; i < N; i++)
    {
        clear();
        robotPtr->yP += inc;
        drawForeground(robotPtr, map);
        sleep(waitTimeMove);
    }
}

void forward(Robot *robotPtr, int** map)
{
    switch (robotPtr->direction)
    {
    case 0:
        moveUp(robotPtr, map);
        break;
    case 1:
        moveEast(robotPtr, map);
        break;
    case 2:
        moveDown(robotPtr, map);
        break;
    case 3:
        moveWest(robotPtr, map);
        break;
    }
}

void drawRobot(Robot *robotPtr)
{
    if (robotPtr->isCarryingMarker == 1)
    {
        setColour(yellow);
    }
    else
    {
        setColour(gray);
    }

    fillOval(robotPtr->xP, robotPtr->yP, squareSideLength, squareSideLength);
    
    // eyes
    setColour(black);
    if (robotPtr->direction == 0)
    {
        fillRect(robotPtr->xP + squareSideLength / 3 - squareSideLength / 5, robotPtr->yP + squareSideLength / 5, squareSideLength / 5, squareSideLength / 5);
        fillRect(robotPtr->xP + 2 * squareSideLength / 3, robotPtr->yP + squareSideLength / 5, squareSideLength / 5, squareSideLength / 5);
    }
    else if (robotPtr->direction == 1)
    {
        fillRect(robotPtr->xP + squareSideLength / 3, robotPtr->yP + squareSideLength / 3, squareSideLength / 5, squareSideLength / 5);
        fillRect(robotPtr->xP + 2 * squareSideLength / 3, robotPtr->yP + squareSideLength / 3, squareSideLength / 5, squareSideLength / 5);
    }
    else if (robotPtr->direction == 2)
    {
        fillRect(robotPtr->xP + squareSideLength / 3, robotPtr->yP + 2 * squareSideLength / 3, squareSideLength / 5, squareSideLength / 5);
        fillRect(robotPtr->xP + 2 * squareSideLength / 3, robotPtr->yP + 2 * squareSideLength / 3, squareSideLength / 5, squareSideLength / 5);
    }
    else if (robotPtr->direction == 3)
    {
        fillRect(robotPtr->xP, robotPtr->yP + squareSideLength / 3, squareSideLength / 5, squareSideLength / 5);
        fillRect(robotPtr->xP + squareSideLength / 3, robotPtr->yP + squareSideLength / 3, squareSideLength / 5, squareSideLength / 5);
    }
    
}


void goHome(Robot *robotPtr, char* moveStack, int noOfMoves, int** map)
{
    char instruct;
    turnAround(robotPtr, map);
    for (int i = noOfMoves - 1; i >= 0; i--)
    {
        instruct = *(moveStack+i);
        switch (instruct)
        {
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

void turnAround(Robot *robotPtr, int** map)
{
    //two rights or lefts make you turn around
    right(robotPtr, map);
    right(robotPtr, map);
}

void moveEast(Robot *robotPtr, int** map)
{
    int N = 5;
    int inc = squareSideLength / N;
    for (int i = 0; i < N; i++)
    {
        clear();
        robotPtr->xP += inc;
        drawForeground(robotPtr, map);
        sleep(waitTimeMove);
    }
}

void moveWest(Robot *robotPtr, int** map)
{
    int N = 5;
    int inc = squareSideLength / N;
    for (int i = 0; i < N; i++)
    {
        clear();
        robotPtr->xP -= inc;
        drawForeground(robotPtr, map);
        sleep(waitTimeMove);
    }
}