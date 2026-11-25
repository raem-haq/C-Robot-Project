#ifndef PLANNER_H
#define PLANNER_H

#include <stdbool.h>

#include "belief.h"
#include "mission.h"
#include "robot.h"

bool exploreAndReturn(Robot *robotPtr, int map[DIMENSIONS][DIMENSIONS],
                      Mission *mission);
void observe(Robot *robotPtr, int groundTruthMap[DIMENSIONS][DIMENSIONS],
             Belief *belief);

#endif
