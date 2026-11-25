#include "planner.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "../constants.h"
#include "../heap.h"

typedef struct {
    Point point;
    int g;
    int f;
} Node;

typedef struct {
    Point previous;
    Direction moveDirection;
    bool hasPrevious;
} Parent;

typedef enum {
    ROUTE_DONE,
    ROUTE_STOPPED_AT_MARKER,
    ROUTE_ABORTED
} RouteResult;

static const int directionX[] = {0, 1, 0, -1};
static const int directionY[] = {-1, 0, 1, 0};

static int compareNodes(const void *a, const void *b);
static int heuristic(Point a, Point b);
static bool inBounds(int x, int y);
static bool samePoint(Point a, Point b);
static Point robotPosition(const Robot *robotPtr);
static int countVisibleUnknown(Point point, Direction direction, const Belief *belief);
static bool aStarRoute(const Belief *belief, Point start, Point goal, Direction *moveDirections,
                       int *moveCount);
static int routeDistance(const Belief *belief, Point start, Point goal);
static bool selectFrontier(Robot *robotPtr, const Belief *belief, Direction *moveDirections,
                           int *moveCount, Direction *finalDirection);
static void turnToDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS], Belief *belief);
static void moveInDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS], Belief *belief,
                            Mission *mission);
static RouteResult executeRoute(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                                Belief *belief, const Direction *moveDirections, int moveCount,
                                bool alignFinalDirection, Direction finalDirection,
                                bool stopAtMarker, bool checkReturnEnergy, Point homePosition,
                                Mission *mission);
static bool returnHome(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                       Belief *belief, Point homePosition, Mission *mission);

bool exploreAndReturn(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS], Mission *mission) {
    assert(robotPtr != NULL);
    assert(map != NULL);
    assert(mission != NULL);
    assert(mission->energy >= 0);
    assert(mission->steps >= 0);
    assert(mission->delivered >= 0);
    Belief belief;
    Direction moveDirections[DIMENSIONS * DIMENSIONS];
    Point homePosition = {robotPtr->xP / SQUARE_SIDE_LENGTH, robotPtr->yP / SQUARE_SIDE_LENGTH};

    belief_init(&belief);
    observe(robotPtr, map, &belief);

    while (true) {
        Point currentPosition = robotPosition(robotPtr);
        int moveCount = 0;
        Direction finalDirection = DIRECTION_NORTH;

        if (!robotPtr->isCarryingMarker &&
            belief.markers[currentPosition.x][currentPosition.y] > 0) {
            robotPtr->isCarryingMarker = true;
            --map[currentPosition.x][currentPosition.y];
            --belief.markers[currentPosition.x][currentPosition.y];
        }

        if (robotPtr->isCarryingMarker) {
            if (!aStarRoute(&belief, currentPosition, homePosition, moveDirections, &moveCount)) {
                fprintf(stderr, "ERROR: Could not find a known route to deliver a marker\n");
                return false;
            }
            executeRoute(robotPtr, map, &belief, moveDirections, moveCount, false, DIRECTION_NORTH,
                         false, false, homePosition, mission);
            robotPtr->isCarryingMarker = false;
            mission->delivered++;
            continue;
        }

        Point nearestMarker = {-1, -1};
        int nearestMarkerDistance = INT_MAX;
        Direction markerRoute[DIMENSIONS * DIMENSIONS];
        for (int x = 0; x < DIMENSIONS; x++) {
            for (int y = 0; y < DIMENSIONS; y++) {
                if (belief.cell[x][y] != FREE || belief.markers[x][y] <= 0) {
                    continue;
                }
                int candidateMoveCount;
                if (aStarRoute(&belief, currentPosition, (Point){x, y}, markerRoute,
                               &candidateMoveCount) &&
                    candidateMoveCount < nearestMarkerDistance) {
                    for (int i = 0; i < candidateMoveCount; i++) {
                        moveDirections[i] = markerRoute[i];
                    }
                    moveCount = candidateMoveCount;
                    nearestMarker = (Point){x, y};
                    nearestMarkerDistance = candidateMoveCount;
                }
            }
        }
        if (nearestMarker.x >= 0) {
            RouteResult result =
                executeRoute(robotPtr, map, &belief, moveDirections, moveCount, false,
                             DIRECTION_NORTH, true, true, homePosition, mission);
            if (result == ROUTE_ABORTED) {
                return returnHome(robotPtr, map, &belief, homePosition, mission);
            }
            continue;
        }

        if (selectFrontier(robotPtr, &belief, moveDirections, &moveCount, &finalDirection)) {
            RouteResult result =
                executeRoute(robotPtr, map, &belief, moveDirections, moveCount, true,
                             finalDirection, true, true, homePosition, mission);
            if (result == ROUTE_ABORTED) {
                return returnHome(robotPtr, map, &belief, homePosition, mission);
            }
            continue;
        }

        if (currentPosition.x == homePosition.x && currentPosition.y == homePosition.y) {
            assert(mission->energy >= 0);
            return true;
        }

        if (!aStarRoute(&belief, currentPosition, homePosition, moveDirections, &moveCount)) {
            fprintf(stderr, "ERROR: Could not find a known route back to the home square\n");
            return false;
        }
        RouteResult result =
            executeRoute(robotPtr, map, &belief, moveDirections, moveCount, false,
                         DIRECTION_NORTH, true, false, homePosition, mission);
        if (result == ROUTE_STOPPED_AT_MARKER) {
            continue;
        }
        assert(mission->energy >= 0);
        return true;
    }
}

static int compareNodes(const void *a, const void *b) {
    const Node *nodeA = a; // the heap library uses void pointers, so we need to convert to Node*.
    const Node *nodeB = b;

    if (nodeA->f != nodeB->f) {
        return nodeA->f < nodeB->f ? -1 : 1;
    }
    if (nodeA->g != nodeB->g) {
        return nodeA->g > nodeB->g ? -1 : 1;
    }
    if (nodeA->point.x != nodeB->point.x) {
        return nodeA->point.x < nodeB->point.x ? -1 : 1;
    }
    if (nodeA->point.y != nodeB->point.y) {
        return nodeA->point.y < nodeB->point.y ? -1 : 1;
    }
    return 0;
}

static int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y); // Manhattan distance
}

static bool inBounds(int x, int y) {
    return x >= 0 && x < DIMENSIONS && y >= 0 && y < DIMENSIONS;
}

static bool samePoint(Point a, Point b) {
    return a.x == b.x && a.y == b.y;
}

static Point robotPosition(const Robot *robotPtr) {
    return (Point){robotPtr->xP / SQUARE_SIDE_LENGTH, robotPtr->yP / SQUARE_SIDE_LENGTH};
}

static int routeDistance(const Belief *belief, Point start, Point goal) {
    Direction moveDirections[DIMENSIONS * DIMENSIONS];
    int moveCount;

    if (!aStarRoute(belief, start, goal, moveDirections, &moveCount)) {
        return INT_MAX;
    }
    return moveCount;
}

void observe(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS], Belief *belief) {
    Point position = robotPosition(robotPtr);
    Direction forward = robotPtr->direction;
    // The direction enum is defined in such a way that this will always give the correct left
    // direction.
    Direction left = (Direction)((forward + 3) % 4);
    Direction right = (Direction)((forward + 1) % 4);
    Point visible[] = {position,
                       {position.x + directionX[forward], position.y + directionY[forward]},
                       {position.x + directionX[left], position.y + directionY[left]},
                       {position.x + directionX[right], position.y + directionY[right]},
                       {position.x + directionX[forward] + directionX[left],
                        position.y + directionY[forward] + directionY[left]},
                       {position.x + directionX[forward] + directionX[right],
                        position.y + directionY[forward] + directionY[right]}};

    for (size_t i = 0; i < sizeof(visible) / sizeof(visible[0]); i++) {
        int x = visible[i].x;
        int y = visible[i].y;
        if (inBounds(x, y)) {
            int truth = groundTruthMap[x][y];
            belief->cell[x][y] = truth == 0 ? BLOCKED : FREE;
            belief->markers[x][y] = truth >= 3 ? truth - 2 : 0;
        }
    }
}

static int countVisibleUnknown(Point point, Direction direction, const Belief *belief) {
    Direction left = (Direction)((direction + 3) % 4);
    Direction right = (Direction)((direction + 1) % 4);
    Point visible[] = {{point.x + directionX[direction], point.y + directionY[direction]},
                       {point.x + directionX[left], point.y + directionY[left]},
                       {point.x + directionX[right], point.y + directionY[right]},
                       {point.x + directionX[direction] + directionX[left],
                        point.y + directionY[direction] + directionY[left]},
                       {point.x + directionX[direction] + directionX[right],
                        point.y + directionY[direction] + directionY[right]}};
    int unknownCount = 0;
    for (size_t i = 0; i < sizeof(visible) / sizeof(visible[0]); i++) {
        int x = visible[i].x;
        int y = visible[i].y;
        if (inBounds(x, y) && belief->cell[x][y] == UNKNOWN) {
            unknownCount++;
        }
    }
    return unknownCount;
}

static bool aStarRoute(const Belief *belief, Point start, Point goal, Direction *moveDirections,
                       int *moveCount) {
    int gScore[DIMENSIONS][DIMENSIONS];
    Parent parent[DIMENSIONS][DIMENSIONS];
    Direction reverseMoveDirections[DIMENSIONS * DIMENSIONS];
    MinHeap openSet;
    Node current;
    Node next;

    *moveCount = 0;
    if (!inBounds(start.x, start.y) || !inBounds(goal.x, goal.y) ||
        belief->cell[start.x][start.y] != FREE || belief->cell[goal.x][goal.y] != FREE) {
        return false;
    }
    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            gScore[x][y] = INT_MAX;
            parent[x][y].hasPrevious = false;
        }
    }

    gScore[start.x][start.y] = 0;
    heap_init(&openSet, DIMENSIONS * DIMENSIONS * 4, sizeof(Node), compareNodes);
    next = (Node){start, 0, heuristic(start, goal)};
    heap_push(&openSet, &next);

    while (!heap_empty(&openSet)) {
        heap_pop(&openSet, &current);
        Point currentPoint = current.point;
        if (current.g != gScore[currentPoint.x][currentPoint.y]) {
            continue;
        }

        if (samePoint(currentPoint, goal)) {
            int reverseCount = 0;
            Point routePoint = goal;
            while (!samePoint(routePoint, start)) {
                Parent routeParent = parent[routePoint.x][routePoint.y];
                if (!routeParent.hasPrevious || reverseCount == DIMENSIONS * DIMENSIONS) {
                    heap_free(&openSet);
                    return false;
                }
                reverseMoveDirections[reverseCount++] = routeParent.moveDirection;
                routePoint = routeParent.previous;
            }
            for (int i = 0; i < reverseCount; i++) {
                moveDirections[i] = reverseMoveDirections[reverseCount - i - 1];
            }
            *moveCount = reverseCount;
            heap_free(&openSet);
            return true;
        }

        for (int direction = 0; direction < 4; direction++) {
            Point nextPoint = {currentPoint.x + directionX[direction],
                               currentPoint.y + directionY[direction]};
            if (!inBounds(nextPoint.x, nextPoint.y) ||
                belief->cell[nextPoint.x][nextPoint.y] != FREE) {
                continue;
            }

            int tentativeG = current.g + 1;
            if (tentativeG < gScore[nextPoint.x][nextPoint.y]) {
                gScore[nextPoint.x][nextPoint.y] = tentativeG;
                parent[nextPoint.x][nextPoint.y] =
                    (Parent){currentPoint, (Direction)direction, true};
                next = (Node){nextPoint, tentativeG, tentativeG + heuristic(nextPoint, goal)};
                heap_push(&openSet, &next);
            }
        }
    }

    heap_free(&openSet);
    return false;
}

static bool selectFrontier(Robot *robotPtr, const Belief *belief, Direction *moveDirections,
                           int *moveCount, Direction *finalDirection) {
    int bestGain = 0;
    int bestCost = 0;
    bool foundFrontier = false;
    Point start = robotPosition(robotPtr);
    Direction candidateMoveDirections[DIMENSIONS * DIMENSIONS];

    *moveCount = 0;
    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            if (belief->cell[x][y] != FREE) {
                continue;
            }
            Point point = {x, y};
            int candidateMoveCount;
            if (!aStarRoute(belief, start, point, candidateMoveDirections, &candidateMoveCount)) {
                continue;
            }

            for (int direction = 0; direction < 4; direction++) {
                int gain = countVisibleUnknown(point, (Direction)direction, belief);
                if (gain == 0) {
                    continue;
                }

                // We add 1 to the denominator to avoid division by zero
                int candidateDenominator = candidateMoveCount + 1;
                int bestDenominator = bestCost + 1;
                // We're effectively comparing gain/cost ratios, but we avoid floating point
                // division by cross-multiplying.
                bool betterScore =
                    !foundFrontier || gain * bestDenominator > bestGain * candidateDenominator;
                bool equalScore =
                    foundFrontier && gain * bestDenominator == bestGain * candidateDenominator;
                if (betterScore || (equalScore && candidateMoveCount < bestCost)) {
                    for (int i = 0; i < candidateMoveCount; i++) {
                        moveDirections[i] = candidateMoveDirections[i];
                    }
                    *moveCount = candidateMoveCount;
                    *finalDirection = (Direction)direction;
                    bestGain = gain;
                    bestCost = candidateMoveCount;
                    foundFrontier = true;
                }
            }
        }
    }
    return foundFrontier;
}

static void turnToDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS], Belief *belief) {
    int turnCount = (requiredDirection - *direction + 4) % 4;
    if (turnCount == 1) {
        right(robotPtr, groundTruthMap);
        *direction = (Direction)((*direction + 1) % 4);
        observe(robotPtr, groundTruthMap, belief);
    } else if (turnCount == 2) {
        right(robotPtr, groundTruthMap);
        *direction = (Direction)((*direction + 1) % 4);
        observe(robotPtr, groundTruthMap, belief);
        right(robotPtr, groundTruthMap);
        *direction = (Direction)((*direction + 1) % 4);
        observe(robotPtr, groundTruthMap, belief);
    } else if (turnCount == 3) {
        left(robotPtr, groundTruthMap);
        *direction = (Direction)((*direction + 3) % 4);
        observe(robotPtr, groundTruthMap, belief);
    }
    *direction = requiredDirection;
}

static void moveInDirection(Robot *robotPtr, Direction *direction, Direction requiredDirection,
                            int groundTruthMap[DIMENSIONS][DIMENSIONS], Belief *belief,
                            Mission *mission) {
    turnToDirection(robotPtr, direction, requiredDirection, groundTruthMap, belief);
    forward(robotPtr, groundTruthMap);
    mission->steps++;
    mission->energy--;
}

static RouteResult executeRoute(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                                Belief *belief, const Direction *moveDirections, int moveCount,
                                bool alignFinalDirection, Direction finalDirection,
                                bool stopAtMarker, bool checkReturnEnergy, Point homePosition,
                                Mission *mission) {
    Direction direction = robotPtr->direction;

    for (int i = 0; i < moveCount; i++) {
        if (checkReturnEnergy) {
            Point here = robotPosition(robotPtr);
            Point next = {here.x + directionX[moveDirections[i]],
                          here.y + directionY[moveDirections[i]]};
            int gHere = routeDistance(belief, here, homePosition);
            int gNext = routeDistance(belief, next, homePosition);
            int gHerePlusOne = gHere == INT_MAX ? INT_MAX : gHere + 1;
            int returnCost = gHerePlusOne < gNext ? gHerePlusOne : gNext;
            if (mission->energy <= returnCost) {
                return ROUTE_ABORTED;
            }
        }

        moveInDirection(robotPtr, &direction, moveDirections[i], groundTruthMap, belief, mission);
        observe(robotPtr, groundTruthMap, belief);
        if (checkReturnEnergy) {
            assert(mission->energy >=
                   routeDistance(belief, robotPosition(robotPtr), homePosition));
        }
        if (stopAtMarker && atMarker(robotPtr, groundTruthMap)) {
            return ROUTE_STOPPED_AT_MARKER;
        }
    }

    if (alignFinalDirection) {
        turnToDirection(robotPtr, &direction, finalDirection, groundTruthMap, belief);
    }
    return ROUTE_DONE;
}

static bool returnHome(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
                       Belief *belief, Point homePosition, Mission *mission) {
    Direction moveDirections[DIMENSIONS * DIMENSIONS];
    int moveCount;
    if (!aStarRoute(belief, robotPosition(robotPtr), homePosition, moveDirections, &moveCount)) {
        fprintf(stderr, "ERROR: Could not find a known route back to the home square\n");
        return false;
    }
    executeRoute(robotPtr, groundTruthMap, belief, moveDirections, moveCount, false,
                 DIRECTION_NORTH, false, false, homePosition, mission);
    assert(mission->energy >= 0);
    return true;
}
