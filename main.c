#include "graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "grid.h"
#include "constants.h"

void addToMoveStack(char**, char, int*, int*);
void DFS(Robot*, int**, int**, int*, char**, int*);
void comeBack(Robot*, char*, int, int**);
int** initG(void);

int main(int argc, char** argv)
{
    int** map = initGrid();
    // if home square coordinares and inital direction are not given.
    int homeXC = 0;
    int homeYC = 0;
    char *typedDirection = "east";
    char initDir = 1;
    if (argc == 4) // Four arguments were typed
    {
    homeXC = atoi(argv[1]); // Get x value
    homeYC = atoi(argv[2]); // Get y value
    typedDirection = argv[3]; // Get direction
    }
    // convert direction from word to integer
    if (strcmp(typedDirection,"north")==0){
        initDir = 0;
    }
    else if (strcmp(typedDirection,"east")==0){
        initDir = 1;
    } else if (strcmp(typedDirection,"south")==0)
    {
        initDir = 2;    
    } else if (strcmp(typedDirection,"west")==0)
    {
        initDir = 3;
    }
    
    addHome(map, homeXC, homeYC);//add the home square

    //declare and initialise the robot
    Robot robot;
    Robot *robotPtr = &robot;
    initRobot(robotPtr, homeXC*squareSideLength, homeYC*squareSideLength, initDir);

    // declare some file pointers
    FILE *markerFPtr;
    FILE *obsFPtr;

    // Open the files in read mode
    markerFPtr = fopen("markers.txt", "r");
    obsFPtr = fopen("obstacles.txt", "r");

    //If any of the files are not found, print an error message
    if ((markerFPtr == NULL) || (obsFPtr == NULL))
    {
        printf("ERROR: FILE NOT FOUND\n");
    }


    int x,y;
    char line[50];//each line is only 4 characters but i've made

    //read from marker file
    while(fgets(line, sizeof(line), markerFPtr)) {
        line[strcspn(line, "\n")] = 0;//remove null character
        sscanf(line, "%d,%d", &x, &y);//extract x and y coordinate of marker
        addMarker(map,x,y);//add marker using function from grid.h
    }   

    //read from obstacle file
    while(fgets(line, sizeof(line), obsFPtr)) {
        line[strcspn(line, "\n")] = 0;//remove null character
        sscanf(line, "%d,%d", &x, &y);//extract x and y coordinate of marker
        addObstacle(map,x,y);//add obstacle using function from grid.h
    }   

    int gridSideLength = squareSideLength * dimensions;
    setWindowSize(gridSideLength, gridSideLength);

    //initialise variables for recording moves
    int noOfMoves = 0;
    int length = dimensions; //records the length of the dynamic array
    char* moveStack = (char*)malloc(length*sizeof(char));//array of moves
    if(moveStack == NULL) {
        printf("ERROR: Memory allocation failed of \"moveStack\"\n");
    }

    
    drawGrid();//draw the grid lines
    drawForeground(robotPtr, map);//draw the robot, obstacles, markers and the home square

    int** isVisitedAt = initG();//for keeping a record of which cells have been visited
    DFS(robotPtr, isVisitedAt, map, &noOfMoves, &moveStack, &length);//my traversal algorithm
    goHome(robotPtr, moveStack, noOfMoves, map);//the robot stops at a random place, but I want to go to the home square

    //free all pointers initalised with malloc
    freeMap(map);
    freeMap(isVisitedAt);
    free(moveStack);

    //close files
    fclose(markerFPtr);
    fclose(obsFPtr);

    return 0;
}


void addToMoveStack(char** moveStack, char move, int* noOfMoves, int* length){
    if (*noOfMoves == *length){ //IF the move-array  is full
        *(length) *= 2;// double the length
        *moveStack = (char*)realloc(*moveStack, (*length)*sizeof(char));//annd create a new array out of the old array with double the length
        if (*moveStack == NULL)
        {
            printf("ERROR: Memory allocation failed of \"*moveStack\" in addToMoveStack\n");
        }
    }
    *(*moveStack + *noOfMoves) = move;//add the new move to the array
    ++*(noOfMoves);//increase the number of moves
}



//this procedure is to come back to a square which had/has a marker after you've gone to the home square
void comeBack(Robot *robotPtr, char* moveStack, int noOfMoves, int** map)
{
    char instruct;
    turnAround(robotPtr, map);
    for (int i = 0; i < noOfMoves; i++)
    {
        instruct = *(moveStack+i);
        switch (instruct)
        {
            case 'F':
                forward(robotPtr, map);
                break;
            case 'L':
                left(robotPtr, map);
                break;
            case 'R':
                right(robotPtr, map);
                break;
            default:
                break;
        }
    }
}

int** initG(void){
    //create a 2D array of dimensions by dimensions
    int** G = malloc(dimensions*sizeof(int*));
    if (G == NULL){
        printf("ERROR: Memory allocation failed of \"G\" in initG\n");
    }
    for (int i = 0; i< dimensions; i++)
    {
        *(G+i) = malloc(dimensions*sizeof(int));
        if (*(G+i) == NULL){
            printf("ERROR: Memory allocation failed of \"*(G+i)\" in initG\n");
        }
        for (int j = 0; j < dimensions; j++){
            *(*(G+i)+j) = 0;//initialise all cells to 0 (unvisited)
        }
    }
    return G;
}

int shouldGoForward(Robot* robotPtr, int** map, int** isVisitedAt)
{
    // you should go forward if you can go forward AND you have NOT visited the square in front 
    int x = robotPtr->xP/squareSideLength;
    int y = robotPtr->yP/squareSideLength;
    int result = canForward(robotPtr, map);
    if (result)
    {
        int haveVisited;
        switch (robotPtr->direction)
            {
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


void backOne(Robot* robotPtr, char* moveStack, int* noOfMoves, int** map)
{
    int haveMovedBack = 0;
    char instruct;
    //reverse all the instructions until you've gone back
    turnAround(robotPtr, map);
    while (!haveMovedBack)
    {
        instruct = *(moveStack+*noOfMoves-1);
        switch (instruct)
        {
            case 'F':
                forward(robotPtr, map);//since you've turned around, moving forward is the same ad having moved back.
                haveMovedBack = 1;
                break;
            case 'L':
                right(robotPtr, map);
                break;
            case 'R':
                left(robotPtr, map);
                break;
            default:
                break;
        }
        --*(noOfMoves);
    }
    turnAround(robotPtr, map);//turn around so you face the same direction
}

void DFS(Robot* robotPtr, int** isVisitedAt, int** map, int* noOfMoves, char** moveStack, int* length){
    int x = robotPtr->xP/squareSideLength;//convert from pixels to coordnates
    int y = robotPtr->yP/squareSideLength;
    isVisitedAt[x][y] = 1;//you've now visited this square
    if (atMarker(robotPtr, map)){
        int noMarkersAtSquare = *(*(map+x)+y)- 2;//3+ means there are markers at that square
        for (int i = 0; i<noMarkersAtSquare;i++)
        {
            robotPtr->isCarryingMarker = 1;//pick up the marker
            --*(*(map+x)+y);//remove one marker from the square
            goHome(robotPtr, *moveStack, *noOfMoves, map);//go home
            robotPtr->isCarryingMarker = 0;//drop the marker
            comeBack(robotPtr, *moveStack, *noOfMoves, map);// and come back to this square
        }
    }
        //loop 4 times for each direction
        for (int i = 0; i< 4; i++){
            if (shouldGoForward(robotPtr, map, isVisitedAt)){//keep recursing until you can't/shouldn't go forwards, e.g., if there is no unvisited squares
                forward(robotPtr, map);
                addToMoveStack(moveStack, 'F', noOfMoves, length);
                DFS(robotPtr, isVisitedAt, map, noOfMoves, moveStack, length);//keep recursing
                backOne(robotPtr, *moveStack, noOfMoves, map);
            }
            left(robotPtr, map);
            addToMoveStack(moveStack, 'L', noOfMoves, length);
        }
}








