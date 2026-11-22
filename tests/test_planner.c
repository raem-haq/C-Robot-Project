#include "../simulation/planner.h"

#include <assert.h>
#include <stdio.h>

static int actionCount;

int atMarker(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int x = robotPtr->xP / SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP / SQUARE_SIDE_LENGTH;
    return map[x][y] >= 3;
}

void left(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    (void)map;
    robotPtr->direction = (Direction)((robotPtr->direction + 3) % 4);
    assert(++actionCount < 5000);
}

void right(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    (void)map;
    robotPtr->direction = (Direction)((robotPtr->direction + 1) % 4);
    assert(++actionCount < 5000);
}

void forward(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    int x = robotPtr->xP / SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP / SQUARE_SIDE_LENGTH;

    switch (robotPtr->direction) {
        case DIRECTION_NORTH:
            --y;
            break;
        case DIRECTION_EAST:
            ++x;
            break;
        case DIRECTION_SOUTH:
            ++y;
            break;
        case DIRECTION_WEST:
            --x;
            break;
    }

    assert(x >= 0 && x < DIMENSIONS && y >= 0 && y < DIMENSIONS);
    assert(map[x][y] != 0);
    robotPtr->xP = x * SQUARE_SIDE_LENGTH;
    robotPtr->yP = y * SQUARE_SIDE_LENGTH;
    assert(++actionCount < 5000);
}

static void runScenario(int markersAtOneCell) {
    int map[DIMENSIONS][DIMENSIONS];
    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            map[x][y] = 0;
        }
    }

    map[0][0] = 1;
    map[1][0] = 2;
    map[2][0] = 2 + markersAtOneCell;
    map[0][1] = 2;
    map[1][1] = 2;
    map[0][2] = 2;
    map[1][2] = 2;

    Robot robot = {0, 0, DIRECTION_NORTH, false};
    actionCount = 0;

    assert(exploreAndReturn(&robot, map));
    assert(robot.xP == 0 && robot.yP == 0);
    assert(!robot.isCarryingMarker);
    assert(map[2][0] == 2);
}

int main(void) {
    runScenario(0);
    runScenario(1);
    runScenario(2);
    puts("planner tests passed");
    return 0;
}
