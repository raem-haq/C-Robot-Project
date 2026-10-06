#include "drawing/graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "simulation/robot.h"
#include "simulation/grid.h"
#include "constants.h"

void addToMoveStack(char**, char, int*, int*);
void DFS(Robot*, int**, int**, int*, char**, int*);
void comeBack(Robot*, char*, int, int**);
int** initG(void);

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

    // declare some file pointers
    FILE *markerFPtr;
    FILE *obsFPtr;

    // Open the files in read mode
    markerFPtr = fopen("data/markers.txt", "r");
    obsFPtr = fopen("data/obstacles.txt", "r");

    //If any of the files are not found, print an error message
    if ((markerFPtr == NULL) || (obsFPtr == NULL)) {
        printf("ERROR: FILE NOT FOUND\n");
    }


    int x,y;
    char line[50];//each line is only 4 characters but i've made

    //read from marker file
    while(fgets(line, sizeof(line), markerFPtr)) {
        line[strcspn(line, "\n")] = 0;//remove null character
        sscanf(line, "%d,%d", &x, &y);//extract x and y coordinate of marker
        addMarker(groundTruthMap,x,y);//add marker using function from grid.h
    }   

    //read from obstacle file
    while(fgets(line, sizeof(line), obsFPtr)) {
        line[strcspn(line, "\n")] = 0;//remove null character
        sscanf(line, "%d,%d", &x, &y);//extract x and y coordinate of marker
        addObstacle(groundTruthMap,x,y);//add obstacle using function from grid.h
    }   

    Pixel gridSideLength = SQUARE_SIDE_LENGTH * DIMENSIONS;
    setWindowSize(gridSideLength, gridSideLength);

    //initialise variables for recording moves
    int noOfMoves = 0;
    int length = DIMENSIONS; //records the length of the dynamic array
    char* moveStack = (char*)malloc(length*sizeof(char));//array of moves
    if(moveStack == NULL) {
        printf("ERROR: Memory allocation failed of \"moveStack\"\n");
    }

    
    drawGrid();//draw the grid lines
    drawForeground(robotPtr, groundTruthMap);//draw the robot, obstacles, markers and the home square

    int** isVisitedAt = initG();//for keeping a record of which cells have been visited
    DFS(robotPtr, isVisitedAt, groundTruthMap, &noOfMoves, &moveStack, &length);//my traversal algorithm
    goHome(robotPtr, moveStack, noOfMoves, groundTruthMap);//the robot stops at a random place, but I want to go to the home square

    //free all pointers initalised with malloc
    freeMap(isVisitedAt);
    free(moveStack);

    //close files
    fclose(markerFPtr);
    fclose(obsFPtr);

    return 0;
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

int** initG(void) {
    //create a 2D array of DIMENSIONS by DIMENSIONS
    int** G = malloc(DIMENSIONS*sizeof(int*));
    if (G == NULL) {
        printf("ERROR: Memory allocation failed of \"G\" in initG\n");
    }
    for (int i = 0; i< DIMENSIONS; i++) {
        *(G+i) = malloc(DIMENSIONS*sizeof(int));
        if (*(G+i) == NULL) {
            printf("ERROR: Memory allocation failed of \"*(G+i)\" in initG\n");
        }
        for (int j = 0; j < DIMENSIONS; j++) {
            *(*(G+i)+j) = 0;//initialise all cells to 0 (unvisited)
        }
    }
    return G;
}

int shouldGoForward(Robot* robotPtr, int** groundTruthMap, int** isVisitedAt) {
    // you should go forward if you can go forward AND you have NOT visited the square in front 
    int x = robotPtr->xP/SQUARE_SIDE_LENGTH;
    int y = robotPtr->yP/SQUARE_SIDE_LENGTH;
    int result = canForward(robotPtr, groundTruthMap);
    if (result) {
        int haveVisited;
        switch (robotPtr->direction) {
            case 0:
                haveVisited = isVisitedAt[x][y - 1];
                break;
            case 1:
                haveVisited = isVisitedAt[x+1][y];
                break;
            case 2:
                haveVisited = isVisitedAt[x][y + 1];
                break;
            case 3:
                haveVisited = isVisitedAt[x-1][y];
                break;
            default:
                break;
        }
        result = !haveVisited;
    }
    return result;    
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

void DFS(Robot* robotPtr, int** isVisitedAt, int** groundTruthMap, int* noOfMoves, char** moveStack, int* length) {
    int x = robotPtr->xP/SQUARE_SIDE_LENGTH;//convert from pixels to coordnates
    int y = robotPtr->yP/SQUARE_SIDE_LENGTH;
    isVisitedAt[x][y] = 1;
    if (atMarker(robotPtr, groundTruthMap)) {
        int noMarkersAtSquare = groundTruthMap[x][y] - 2;//3+ means there are markers at that square
        for (int i = 0; i<noMarkersAtSquare;i++) {
            robotPtr->isCarryingMarker = 1;//pick up the marker
            --groundTruthMap[x][y];//remove one marker from the square
            goHome(robotPtr, *moveStack, *noOfMoves, groundTruthMap);//go home
            robotPtr->isCarryingMarker = 0;//drop the marker
            comeBack(robotPtr, *moveStack, *noOfMoves, groundTruthMap);// and come back to this square
        }
    }
        //loop 4 times for each direction
        for (int i = 0; i< 4; i++) {
            if (shouldGoForward(robotPtr, groundTruthMap, isVisitedAt)) {//keep recursing until you can't/shouldn't go forwards, e.g., if there is no unvisited squares
                forward(robotPtr, groundTruthMap);
                addToMoveStack(moveStack, 'F', noOfMoves, length);
                DFS(robotPtr, isVisitedAt, groundTruthMap, noOfMoves, moveStack, length);//keep recursing
                backOne(robotPtr, *moveStack, noOfMoves, groundTruthMap);
            }
            left(robotPtr, groundTruthMap);
            addToMoveStack(moveStack, 'L', noOfMoves, length);
        }
}


void aStarReturn(Robot* robotPtr) {
    (void)robotPtr;
}

void initKnown(KnownCell belief[DIMENSIONS][DIMENSIONS]) {
    for (int i = 0; i< DIMENSIONS; i++) {
        for (int j = 0; j < DIMENSIONS; j++) {
            belief[i][j] = (KnownCell){UNKNOWN, 0, 1.0f};
        }
    }
}
