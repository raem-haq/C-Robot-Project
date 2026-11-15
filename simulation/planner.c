#include "planner.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#include "../constants.h"
#include "../heap.h"

typedef enum { UNKNOWN, FREE, BLOCKED } Knowledge;

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    Point point;
    int g;
    int f;
} Node;

static bool addToMoveStack(char **moveStack, char move, int *moveCount, int *capacity);
static bool DFS(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                Knowledge beliefMap[DIMENSIONS][DIMENSIONS], Point homePosition,
                int *moveCount, char **moveStack, int *capacity);
static void retraceAStarRoute(Robot *robotPtr, const char *moves, int moveCount,
                              Direction markerDirection,
                              int groundTruthMap[DIMENSIONS][DIMENSIONS]);
static void turnToDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS]);
static void moveInDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS]);
static bool shouldGoForward(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                            Knowledge beliefMap[DIMENSIONS][DIMENSIONS]);
static void backOne(Robot *robotPtr, char *moveStack, int *moveCount,
                    int groundTruthMap[DIMENSIONS][DIMENSIONS]);
static void updateBeliefMap(Knowledge beliefMap[DIMENSIONS][DIMENSIONS], int x, int y);
static int compareNodes(const void *a, const void *b);
static int heuristic(Point a, Point b);
static bool inBounds(int x, int y);
static bool aStar(Robot *robotPtr, Knowledge beliefMap[DIMENSIONS][DIMENSIONS],
                  Point homePosition, char *moves, int *moveCount);
static bool aStarReturn(Robot *robotPtr, Knowledge beliefMap[DIMENSIONS][DIMENSIONS],
                        int groundTruthMap[DIMENSIONS][DIMENSIONS], Point homePosition,
                        char *moves, int *moveCount);
static void initKnown(Knowledge beliefMap[DIMENSIONS][DIMENSIONS]);

bool exploreAndReturn(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]) {
    Knowledge beliefMap[DIMENSIONS][DIMENSIONS];
    initKnown(beliefMap);

    int moveCount = 0;
    int capacity = DIMENSIONS;
    char *moveStack = malloc((size_t)capacity * sizeof(*moveStack));
    if (moveStack == NULL) {
        fprintf(stderr, "ERROR: Could not allocate move stack\n");
        return false;
    }

    Point homePosition = {
        robotPtr->xP / SQUARE_SIDE_LENGTH,
        robotPtr->yP / SQUARE_SIDE_LENGTH
    };
    bool success = DFS(robotPtr, map, beliefMap, homePosition,
                       &moveCount, &moveStack, &capacity);
    if (success) {
        goHome(robotPtr, moveStack, moveCount, map);
    } else {
        fprintf(stderr, "ERROR: Exploration failed (could not grow move stack or find a return route)\n");
    }
    free(moveStack);
    return success;
}

static bool addToMoveStack(char **moveStack, char move, int *moveCount, int *capacity) {
    if (*moveCount == *capacity) {
        if (*capacity > INT_MAX / 2) {
            return false;
        }
        int newCapacity = *capacity * 2;
        char *newStack = realloc(*moveStack, (size_t)newCapacity * sizeof(**moveStack));
        if (newStack == NULL) {
            return false;
        }
        *moveStack = newStack;
        *capacity = newCapacity;
    }
    (*moveStack)[(*moveCount)++] = move;
    return true;
}

static void retraceAStarRoute(Robot *robotPtr, const char *moves, int moveCount,
                              Direction markerDirection,
                              int groundTruthMap[DIMENSIONS][DIMENSIONS]) {
    Direction direction = robotPtr->direction;
    for (int i = moveCount - 1; i >= 0; i--) {
        switch (moves[i]) {
            case 'N':
                moveInDirection(robotPtr, &direction, DIRECTION_SOUTH, groundTruthMap);
                break;
            case 'E':
                moveInDirection(robotPtr, &direction, DIRECTION_WEST, groundTruthMap);
                break;
            case 'S':
                moveInDirection(robotPtr, &direction, DIRECTION_NORTH, groundTruthMap);
                break;
            case 'W':
                moveInDirection(robotPtr, &direction, DIRECTION_EAST, groundTruthMap);
                break;
            default:
                break;
        }
    }
    turnToDirection(robotPtr, &direction, markerDirection, groundTruthMap);
}

static void turnToDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS]) {
    int turnCount = (requiredDirection - *direction + 4) % 4;
    if (turnCount == 1) {
        right(robotPtr, groundTruthMap);
    } else if (turnCount == 2) {
        right(robotPtr, groundTruthMap);
        right(robotPtr, groundTruthMap);
    } else if (turnCount == 3) {
        left(robotPtr, groundTruthMap);
    }
    *direction = requiredDirection;
}

static void moveInDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS]) {
    turnToDirection(robotPtr, direction, requiredDirection, groundTruthMap);
    forward(robotPtr, groundTruthMap);
}

static bool shouldGoForward(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                            Knowledge beliefMap[DIMENSIONS][DIMENSIONS]) {
    int x = robotPtr->xP / SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP / SQUARE_SIDE_LENGTH;
    int nextX = x;
    int nextY = y;

    switch (robotPtr->direction) {
        case DIRECTION_NORTH:
            --nextY;
            break;
        case DIRECTION_EAST:
            ++nextX;
            break;
        case DIRECTION_SOUTH:
            ++nextY;
            break;
        case DIRECTION_WEST:
            --nextX;
            break;
        default:
            return false;
    }

    if (!canForward(robotPtr, groundTruthMap)) {
        if (inBounds(nextX, nextY)) {
            beliefMap[nextX][nextY] = BLOCKED;
        }
        return false;
    }
    return beliefMap[nextX][nextY] == UNKNOWN;
}

static void backOne(Robot *robotPtr, char *moveStack, int *moveCount,
                    int groundTruthMap[DIMENSIONS][DIMENSIONS]) {
    turnAround(robotPtr, groundTruthMap);
    while (true) {
        switch (moveStack[*moveCount - 1]) {
            case 'F':
                forward(robotPtr, groundTruthMap);
                --*moveCount;
                turnAround(robotPtr, groundTruthMap);
                return;
            case 'L':
                right(robotPtr, groundTruthMap);
                break;
            case 'R':
                left(robotPtr, groundTruthMap);
                break;
            default:
                break;
        }
        --*moveCount;
    }
}

static void updateBeliefMap(Knowledge beliefMap[DIMENSIONS][DIMENSIONS], int x, int y) {
    beliefMap[x][y] = FREE;
}

static bool DFS(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                Knowledge beliefMap[DIMENSIONS][DIMENSIONS], Point homePosition,
                int *moveCount, char **moveStack, int *capacity) {
    int x = robotPtr->xP / SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP / SQUARE_SIDE_LENGTH;
    updateBeliefMap(beliefMap, x, y);

    if (atMarker(robotPtr, groundTruthMap)) {
        int markersAtSquare = groundTruthMap[x][y] - 2;
        for (int i = 0; i < markersAtSquare; i++) {
            char returnMoves[DIMENSIONS * DIMENSIONS];
            int returnMoveCount = 0;
            Direction markerDirection = robotPtr->direction;

            robotPtr->isCarryingMarker = true;
            --groundTruthMap[x][y];
            if (!aStarReturn(robotPtr, beliefMap, groundTruthMap, homePosition,
                             returnMoves, &returnMoveCount)) {
                robotPtr->isCarryingMarker = false;
                return false;
            }
            robotPtr->isCarryingMarker = false;
            retraceAStarRoute(robotPtr, returnMoves, returnMoveCount, markerDirection,
                              groundTruthMap);
        }
    }

    for (int i = 0; i < 4; i++) {
        if (shouldGoForward(robotPtr, groundTruthMap, beliefMap)) {
            forward(robotPtr, groundTruthMap);
            if (!addToMoveStack(moveStack, 'F', moveCount, capacity) ||
                !DFS(robotPtr, groundTruthMap, beliefMap, homePosition,
                     moveCount, moveStack, capacity)) {
                return false;
            }
            backOne(robotPtr, *moveStack, moveCount, groundTruthMap);
        }
        left(robotPtr, groundTruthMap);
        if (!addToMoveStack(moveStack, 'L', moveCount, capacity)) {
            return false;
        }
    }
    return true;
}

static int compareNodes(const void *a, const void *b) {
    const Node *nodeA = a;
    const Node *nodeB = b;

    if (nodeA->f < nodeB->f) {
        return -1;
    }
    if (nodeA->f > nodeB->f) {
        return 1;
    }
    return 0;
}

static int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

static bool inBounds(int x, int y) {
    return x >= 0 && x < DIMENSIONS && y >= 0 && y < DIMENSIONS;
}

static bool aStar(Robot *robotPtr, Knowledge beliefMap[DIMENSIONS][DIMENSIONS],
                  Point homePosition, char *moves, int *moveCount) {
    Point start = {
        robotPtr->xP / SQUARE_SIDE_LENGTH,
        robotPtr->yP / SQUARE_SIDE_LENGTH
    };
    int gScore[DIMENSIONS][DIMENSIONS];
    Point parent[DIMENSIONS][DIMENSIONS];
    MinHeap openSet;
    Node current;
    Node next;
    int neighborX[] = {0, 1, 0, -1};
    int neighborY[] = {-1, 0, 1, 0};

    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            gScore[x][y] = INT_MAX;
            parent[x][y] = (Point){-1, -1};
        }
    }

    gScore[start.x][start.y] = 0;
    heap_init(&openSet, DIMENSIONS * DIMENSIONS, sizeof(Node), compareNodes);
    next.point = start;
    next.g = 0;
    next.f = heuristic(start, homePosition);
    heap_push(&openSet, &next);

    while (!heap_empty(&openSet)) {
        heap_pop(&openSet, &current);
        if (current.g != gScore[current.point.x][current.point.y]) {
            continue;
        }

        if (current.point.x == homePosition.x && current.point.y == homePosition.y) {
            Point routePoint = current.point;
            int pathLength = 0;
            Point path[DIMENSIONS * DIMENSIONS];

            while (routePoint.x != -1 && routePoint.y != -1) {
                path[pathLength++] = routePoint;
                routePoint = parent[routePoint.x][routePoint.y];
            }

            for (int i = pathLength - 1; i > 0; i--) {
                int xDelta = path[i - 1].x - path[i].x;
                int yDelta = path[i - 1].y - path[i].y;
                if (yDelta == -1 && xDelta == 0) {
                    moves[(*moveCount)++] = 'N';
                } else if (xDelta == 1 && yDelta == 0) {
                    moves[(*moveCount)++] = 'E';
                } else if (yDelta == 1 && xDelta == 0) {
                    moves[(*moveCount)++] = 'S';
                } else if (xDelta == -1 && yDelta == 0) {
                    moves[(*moveCount)++] = 'W';
                }
            }

            heap_free(&openSet);
            return true;
        }

        for (int i = 0; i < 4; i++) {
            int x = current.point.x + neighborX[i];
            int y = current.point.y + neighborY[i];
            int tentativeG;

            if (!inBounds(x, y) || beliefMap[x][y] != FREE) {
                continue;
            }

            tentativeG = current.g + 1;
            if (tentativeG < gScore[x][y]) {
                gScore[x][y] = tentativeG;
                parent[x][y] = current.point;
                next.point = (Point){x, y};
                next.g = tentativeG;
                next.f = tentativeG + heuristic((Point){x, y}, homePosition);
                heap_push(&openSet, &next);
            }
        }
    }

    heap_free(&openSet);
    return false;
}

static bool aStarReturn(Robot *robotPtr, Knowledge beliefMap[DIMENSIONS][DIMENSIONS],
                        int groundTruthMap[DIMENSIONS][DIMENSIONS], Point homePosition,
                        char *moves, int *moveCount) {
    Direction direction = robotPtr->direction;

    if (!aStar(robotPtr, beliefMap, homePosition, moves, moveCount)) {
        return false;
    }

    for (int i = 0; i < *moveCount; i++) {
        Direction requiredDirection;
        switch (moves[i]) {
            case 'N':
                requiredDirection = DIRECTION_NORTH;
                break;
            case 'E':
                requiredDirection = DIRECTION_EAST;
                break;
            case 'S':
                requiredDirection = DIRECTION_SOUTH;
                break;
            case 'W':
                requiredDirection = DIRECTION_WEST;
                break;
            default:
                continue;
        }

        moveInDirection(robotPtr, &direction, requiredDirection, groundTruthMap);
    }
    return true;
}

static void initKnown(Knowledge beliefMap[DIMENSIONS][DIMENSIONS]) {
    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            beliefMap[x][y] = UNKNOWN;
        }
    }
}
