#include "drawing/graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "simulation/robot.h"
#include "simulation/grid.h"
#include "constants.h"
#include <stdbool.h>

void addToMoveStack(char**, char, int*, int*);
void DFS(Robot*, int[DIMENSIONS][DIMENSIONS], KnownCell[DIMENSIONS][DIMENSIONS], int*, char**, int*);
void comeBack(Robot*, char*, int, int**);
void initVisited(bool[DIMENSIONS][DIMENSIONS]);
bool loadLocations(int**, const char*, void (*)(int**, int, int));

typedef enum { UNKNOWN, FREE, BLOCKED } Knowledge;

typedef struct {
    Knowledge state;
    int markers;      // believed marker count, valid once seen
    float cost;       // believed move cost (currently always 1.0)
} KnownCell;
void initKnown(KnownCell[DIMENSIONS][DIMENSIONS]);


int main(int argc, char** argv) {
    KnownCell robotBeliefMap[DIMENSIONS][DIMENSIONS];
    initKnown(robotBeliefMap);

    int** groundTruthMap = initGrid();
    
    int homeX = 0;
    int homeY = 0;
    char *typedDirection = "east";
    char initDir = 1;

    if (argc == 4) {
        homeX = atoi(argv[1]);
        homeY = atoi(argv[2]);
        typedDirection = argv[3];
    }

    if (strcmp(typedDirection,"north")==0) {
        initDir = 0;
    } else if (strcmp(typedDirection,"east")==0) {
        initDir = 1;
    } else if (strcmp(typedDirection,"south")==0) {
        initDir = 2;    
    } else if (strcmp(typedDirection,"west")==0) {
        initDir = 3;
    } // else??
    
    addHome(groundTruthMap, homeX, homeY);

    Robot robot;
    Robot *robotPtr = &robot;
    Pixel homeXP = homeX * SQUARE_SIDE_LENGTH;
    Pixel homeYP = homeY * SQUARE_SIDE_LENGTH;
    initRobot(robotPtr, homeXP, homeYP, initDir);

    if (!loadLocations(groundTruthMap, "data/markers.txt", addMarker) ||
        !loadLocations(groundTruthMap, "data/obstacles.txt", addObstacle)) {
        return EXIT_FAILURE;
    }

    Pixel gridSideLength = SQUARE_SIDE_LENGTH * DIMENSIONS;
    setWindowSize(gridSideLength, gridSideLength);

    
    drawGrid();//draw the grid lines
    drawForeground(robotPtr, groundTruthMap);//draw the robot, obstacles, markers and the home square

    int noOfMoves = 0;
    int length = DIMENSIONS; //records the length of the dynamic array
    char* moveStack = (char*)malloc(length*sizeof(char)); //array of moves
    if(moveStack == NULL) {
        printf("ERROR: Memory allocation failed of \"moveStack\"\n");
        return EXIT_FAILURE;
    }
    
    DFS(robotPtr, groundTruthMap, robotBeliefMap, &noOfMoves, &moveStack, &length);
    //the robot stops at a random place, but I want to go to the home square
    goHome(robotPtr, moveStack, noOfMoves, groundTruthMap);

    free(moveStack);

    return 0;
}


bool loadLocations(int** map, const char* filePath, void (*addLocation)(int**, int, int)) {
    FILE *file = fopen(filePath, "r");
    if (file == NULL) {
        fprintf(stderr, "ERROR: Could not open %s\n", filePath);
        return false;
    }

    char line[50];
    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\n")] = '\0';
        int x, y;
        if (sscanf(line, "%d,%d", &x, &y) != 2 ||
            x < 0 || x >= DIMENSIONS || y < 0 || y >= DIMENSIONS) {
            fprintf(stderr, "ERROR: Invalid coordinates in %s: %s\n", filePath, line);
            return false;
        }
        addLocation(map, x, y);
    }

    if (ferror(file)) {
        fprintf(stderr, "ERROR: Could not read %s\n", filePath);
        return false;
    }
    if (fclose(file) != 0) {
        fprintf(stderr, "ERROR: Could not close %s\n", filePath);
        return false;
    }
    return true;
}

void addToMoveStack(char** moveStack, char move, int* noOfMoves, int* length) {
    if (*noOfMoves == *length) { //IF the move-array  is full
        *(length) *= 2;// double the length
        *moveStack = (char*)realloc(*moveStack, (*length)*sizeof(char));//annd create a new array out of the old array with double the length
        if (*moveStack == NULL) {
            printf("ERROR: Memory allocation failed of \"*moveStack\" in addToMoveStack\n");
        }
    }
    *(*moveStack + *noOfMoves) = move;//add the new move to the array
    ++*(noOfMoves);//increase the number of moves
}



//this procedure is to come back to a square which had/has a marker after you've gone to the home square
void comeBack(Robot *robotPtr, char* moveStack, int noOfMoves, int** groundTruthMap) {
    char instruct;
    turnAround(robotPtr, groundTruthMap);
    for (int i = 0; i < noOfMoves; i++) {
        instruct = *(moveStack+i);
        switch (instruct) {
            case 'F':
                forward(robotPtr, groundTruthMap);
                break;
            case 'L':
                left(robotPtr, groundTruthMap);
                break;
            case 'R':
                right(robotPtr, groundTruthMap);
                break;
            default:
                break;
        }
    }
}

void initVisited(bool visited[DIMENSIONS][DIMENSIONS]) {
    for (int i = 0; i< DIMENSIONS; i++) {
        for (int j = 0; j < DIMENSIONS; j++) {
            visited[i][j] = false;
        }
    }
}

int shouldGoForward(Robot* robotPtr, int** groundTruthMap, KnownCell beliefMap[DIMENSIONS][DIMENSIONS]) {
    // you should go forward if you can go forward AND you have NOT visited the square in front 
    int x = robotPtr->xP/SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP/SQUARE_SIDE_LENGTH;
    int result = canForward(robotPtr, groundTruthMap);
    if (result) {
        int haveVisited;
        switch (robotPtr->direction) {
            case 0:
                return beliefMap[x][y - 1].state == UNKNOWN;
            case 1:
                return beliefMap[x+1][y].state == UNKNOWN;
            case 2:
                return beliefMap[x-1][y].state == UNKNOWN;
            case 3:
                return beliefMap[x][y + 1].state == UNKNOWN;
            default:
                break;
        }
    }
    return 0;    
}


void backOne(Robot* robotPtr, char* moveStack, int* noOfMoves, int** groundTruthMap) {
    int haveMovedBack = 0;
    char instruct;
    //reverse all the instructions until you've gone back
    turnAround(robotPtr, groundTruthMap);
    while (!haveMovedBack) {
        instruct = *(moveStack+*noOfMoves-1);
        switch (instruct) {
            case 'F':
                forward(robotPtr, groundTruthMap);//since you've turned around, moving forward is the same ad having moved back.
                haveMovedBack = 1;
                break;
            case 'L':
                right(robotPtr, groundTruthMap);
                break;
            case 'R':
                left(robotPtr, groundTruthMap);
                break;
            default:
                break;
        }
        --*(noOfMoves);
    }
    turnAround(robotPtr, groundTruthMap);//turn around so you face the same direction
}

void updateBeliefMap(KnownCell beliefMap[DIMENSIONS][DIMENSIONS], int groundTruthMap[DIMENSIONS][DIMENSIONS], int x, int y){
    beliefMap[x][y] = (KnownCell){groundTruthMap[x][y] == 2 ? BLOCKED : FREE, groundTruthMap[x][y] >= 3 ? groundTruthMap[x][y] - 2 : 0,  1.0f};
}

void DFS(Robot* robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS], 
            KnownCell beliefMap[DIMENSIONS][DIMENSIONS], int* noOfMoves, char** moveStack, int* length) {
    int x = robotPtr->xP/SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP/SQUARE_SIDE_LENGTH;
    updateBeliefMap(beliefMap, groundTruthMap, x, y);
    if (atMarker(robotPtr, groundTruthMap)) {
        int noMarkersAtSquare = groundTruthMap[x][y] - 2;//3+ means there are markers at that square
        for (int i = 0; i<noMarkersAtSquare;i++) {
            robotPtr->isCarryingMarker = 1;
            --groundTruthMap[x][y];
            aStarReturn(robotPtr, beliefMap);
            robotPtr->isCarryingMarker = 0;//drop the marker
            comeBack(robotPtr, *moveStack, *noOfMoves, groundTruthMap);// and come back to this square
        }
    }
        //loop 4 times for each direction
        for (int i = 0; i< 4; i++) {
            if (shouldGoForward(robotPtr, groundTruthMap, beliefMap)) {
                //keep recursing until you can't/shouldn't go forwards, e.g., if there is no unvisited squares
                forward(robotPtr, groundTruthMap);
                addToMoveStack(moveStack, 'F', noOfMoves, length);
                DFS(robotPtr, groundTruthMap, beliefMap, noOfMoves, moveStack, length);//keep recursing
                backOne(robotPtr, *moveStack, noOfMoves, groundTruthMap);
            }
            left(robotPtr, groundTruthMap);
            addToMoveStack(moveStack, 'L', noOfMoves, length);
        }
}

typedef struct {
    int row;
    int col;
} Point;

typedef struct {
    Point point;
    int g;  // cost from start
    int f;  // g + heuristic
} Node;

void aStarReturn(Robot* robotPtr, KnownCell beliefMap[DIMENSIONS][DIMENSIONS]) {
    return;
}

void initKnown(KnownCell belief[DIMENSIONS][DIMENSIONS]) {
    for (int i = 0; i< DIMENSIONS; i++) {
        for (int j = 0; j < DIMENSIONS; j++) {
            belief[i][j] = (KnownCell){UNKNOWN, 0, 1.0f};
        }
    }
}
