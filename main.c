#include "constants.h"
#include "drawing/graphics.h"
#include "simulation/grid.h"
#include "simulation/planner.h"
#include "simulation/robot.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool loadLocations(int[DIMENSIONS][DIMENSIONS], const char *,
                   void (*)(int[DIMENSIONS][DIMENSIONS], int, int));

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);

    int groundTruthMap[DIMENSIONS][DIMENSIONS];
    initGrid(groundTruthMap);

    int homeX = 0;
    int homeY = 0;
    char *typedDirection = "east";
    Direction initDir = DIRECTION_EAST;

    if (argc == 4) {
        homeX = atoi(argv[1]);
        homeY = atoi(argv[2]);
        typedDirection = argv[3];
    }

    if (strcmp(typedDirection, "north") == 0) {
        initDir = DIRECTION_NORTH;
    } else if (strcmp(typedDirection, "east") == 0) {
        initDir = DIRECTION_EAST;
    } else if (strcmp(typedDirection, "south") == 0) {
        initDir = DIRECTION_SOUTH;
    } else if (strcmp(typedDirection, "west") == 0) {
        initDir = DIRECTION_WEST;
    } else {
        fprintf(stderr,
                "ERROR: invalid direction '%s'. Use north, east, south, or west (lowercase).\n",
                typedDirection);
        return EXIT_FAILURE;
    }

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

    drawGrid(); // draw the grid lines
    drawForeground(robotPtr,
                   groundTruthMap); // draw the robot, obstacles, markers and the home square

    Mission mission = {0};
    bool success = exploreAndReturn(robotPtr, groundTruthMap, &mission);
    waitForClose();

    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

bool loadLocations(int map[DIMENSIONS][DIMENSIONS], const char *filePath,
                   void (*addLocation)(int[DIMENSIONS][DIMENSIONS], int, int)) {
    FILE *file = fopen(filePath, "r");
    if (file == NULL) {
        fprintf(stderr, "ERROR: Could not open %s\n", filePath);
        return false;
    }

    char line[50];
    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\n")] = '\0';
        int x, y;
        if (sscanf(line, "%d,%d", &x, &y) != 2 || x < 0 || x >= DIMENSIONS || y < 0 ||
            y >= DIMENSIONS) {
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
