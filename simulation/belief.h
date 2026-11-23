#ifndef BELIEF_H
#define BELIEF_H

#include "../constants.h"

typedef enum { UNKNOWN, FREE, BLOCKED } Knowledge;

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    Knowledge cell[DIMENSIONS][DIMENSIONS];
    int markers[DIMENSIONS][DIMENSIONS];
} Belief;

void belief_init(Belief *belief);

#endif
