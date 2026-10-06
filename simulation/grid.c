#include "../drawing/graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../constants.h"

void displayBlocksAndGoals(int **);
void drawGrid(void);

int** initGrid(void);

void addObstacle(int**, int, int);
void addMarker(int**, int, int);
void freeMap(int**);
void addHome(int**, int, int);

/*
map[x][y] -- meaning
2 -- nothing
1 -- home
0 -- obstacle
3+ -- 1+ marker
*/
int** initGrid(void){
    int** map = malloc(DIMENSIONS*sizeof(int*));
    if (map == NULL)
    {
        printf("ERROR: Memory allocation of \"map\" failed\n");
    }
    for (int i = 0; i < DIMENSIONS; i++)
    {
        *(map + i) = malloc(DIMENSIONS * sizeof(int));
        if (*(map + i) == NULL)
        {
            printf("ERROR: Memory allocation of \"(map+%d)\" failed\n", i);
        }
        for (int j = 0; j < DIMENSIONS; j++)
        {
            *(*(map + i) + j) = 2;
        }
    }
    return map;
}



void freeMap(int** map){
    for (int i = 0; i< DIMENSIONS; i++){
        free(*(map+i));
    }
    free(map);
}

/*
map[x][y] -- meaning -- colour
3+ -- 1+ marker -- grey for one marker, yellow for multiple
2 -- nothing -- white
1 -- home -- green
0 -- obstacle -- black
*/
void displayBlocksAndGoals(int **map)
{
    for (int i = 0; i < DIMENSIONS; i++)
    {
        for (int j = 0; j < DIMENSIONS; j++)
        {
            if (*(*(map + i) + j) == 1){
                setColour(green);
                fillRect(SQUARE_SIDE_LENGTH * i, SQUARE_SIDE_LENGTH * j, SQUARE_SIDE_LENGTH, SQUARE_SIDE_LENGTH);
            }            
            else if (*(*(map + i) + j) == 0)
            {
                setColour(black);
                fillRect(SQUARE_SIDE_LENGTH * i, SQUARE_SIDE_LENGTH * j, SQUARE_SIDE_LENGTH, SQUARE_SIDE_LENGTH);
            }
            else if (*(*(map + i) + j) == 3)
            {
                setColour(lightgray);
                fillRect(SQUARE_SIDE_LENGTH * i, SQUARE_SIDE_LENGTH * j, SQUARE_SIDE_LENGTH, SQUARE_SIDE_LENGTH);
            }
            else if (*(*(map + i) + j) > 3)
            {
                setColour(yellow);
                fillRect(SQUARE_SIDE_LENGTH * i, SQUARE_SIDE_LENGTH * j, SQUARE_SIDE_LENGTH, SQUARE_SIDE_LENGTH);
            }
        }
    }
}


void drawGrid()
{
    background();
    setColour(black);
    for (int i = 0; i < DIMENSIONS; i++)
    {
        for (int j = 0; j < DIMENSIONS; j++)
        {
            drawRect(SQUARE_SIDE_LENGTH * i, SQUARE_SIDE_LENGTH * j, SQUARE_SIDE_LENGTH, SQUARE_SIDE_LENGTH);
        }
    }
    foreground();
}

void addMarker(int** map, int x, int y)
{
    switch (map[x][y])
    {
    case 1:
        printf("ERROR: CANNOT PLACE MARKER ON HOME SQUARE\n");
        break;
    case 0:
        printf("ERROR: CANNOT PLACE MARKER ON OBSTACLE\n");
        break;
    default:
        ++map[x][y];
        break;
    }
}



void addObstacle(int** map, int x, int y){
    switch (map[x][y])
    {
    case 2:
        map[x][y] = 0;//0 for obstacle
        break;
    case 1:
        printf("ERROR: CANNOT PLACE OBSTACLE ON HOME SQUARE\n");
        break;
    case 0:
        break;
    default:
        printf("ERROR: CANNOT PLACE OBSTACLE ON MARKER\n");
        break;
    }
}

void addHome(int** map, int x, int y){
    map[x][y] = 1; //happens before adding obstacles and markers so it doesn't need switch-cases
}