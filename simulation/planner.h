#ifndef PLANNER_H
#define PLANNER_H

#include <stdbool.h>

#include "robot.h"

bool exploreAndReturn(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS]);

#endif
