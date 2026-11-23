#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../constants.h"
#include "../simulation/planner.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

static void watchdog_start(unsigned int seconds) {
    alarm(seconds);
}

static void watchdog_stop(void) {
    alarm(0);
}

static void test_smoke(void) {
    watchdog_start(10);
    CHECK(1 == 1);
    watchdog_stop();
}

static void test_observe_updates_belief_and_markers(void) {
    int map[DIMENSIONS][DIMENSIONS];
    Belief belief;
    Robot robot;
    const int x = DIMENSIONS / 2;
    const int y = DIMENSIONS / 2;

    for (int mapX = 0; mapX < DIMENSIONS; mapX++) {
        for (int mapY = 0; mapY < DIMENSIONS; mapY++) {
            map[mapX][mapY] = 2;
        }
    }

    map[x][y] = 1;
    map[x + 1][y] = 5;
    map[x][y - 1] = 3;
    map[x][y + 1] = 2;
    map[x + 1][y - 1] = 0;
    map[x + 1][y + 1] = 4;

    belief_init(&belief);
    initRobot(&robot, x * SQUARE_SIDE_LENGTH, y * SQUARE_SIDE_LENGTH, DIRECTION_EAST);
    observe(&robot, map, &belief);

    for (int mapX = 0; mapX < DIMENSIONS; mapX++) {
        for (int mapY = 0; mapY < DIMENSIONS; mapY++) {
            if (belief.cell[mapX][mapY] == BLOCKED) {
                CHECK(map[mapX][mapY] == 0);
            }
            if (belief.cell[mapX][mapY] == FREE) {
                CHECK(map[mapX][mapY] != 0);
            }
        }
    }

    CHECK(belief.cell[x][y] == FREE);
    CHECK(belief.cell[x + 1][y] == FREE);
    CHECK(belief.cell[x][y - 1] == FREE);
    CHECK(belief.cell[x][y + 1] == FREE);
    CHECK(belief.cell[x + 1][y - 1] == BLOCKED);
    CHECK(belief.cell[x + 1][y + 1] == FREE);
    CHECK(belief.cell[x - 1][y] == UNKNOWN);

    CHECK(belief.markers[x][y] == 0);
    CHECK(belief.markers[x + 1][y] == 3);
    CHECK(belief.markers[x][y - 1] == 1);
    CHECK(belief.markers[x][y + 1] == 0);
    CHECK(belief.markers[x + 1][y - 1] == 0);
    CHECK(belief.markers[x + 1][y + 1] == 2);
    CHECK(belief.markers[x - 1][y] == 0);
}

int main(void) {
    test_smoke();
    test_observe_updates_belief_and_markers();
    puts("tests passed");
    return EXIT_SUCCESS;
}
