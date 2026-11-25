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

static void map_from_ascii(const char rows[DIMENSIONS][DIMENSIONS + 1],
                           int map[DIMENSIONS][DIMENSIONS], Robot *robot) {
    int homeX = -1;
    int homeY = -1;

    for (int y = 0; y < DIMENSIONS; y++) {
        for (int x = 0; x < DIMENSIONS; x++) {
            char cell = rows[y][x];
            if (cell == '#') {
                map[x][y] = 0;
            } else if (cell == '.') {
                map[x][y] = 2;
            } else if (cell == 'H') {
                CHECK(homeX < 0);
                homeX = x;
                homeY = y;
                map[x][y] = 1;
            } else if (cell >= '1' && cell <= '9') {
                map[x][y] = cell - '0' + 2;
            } else {
                CHECK(0 && "invalid ASCII map character");
            }
        }
    }

    CHECK(homeX >= 0);
    initRobot(robot, homeX * SQUARE_SIDE_LENGTH, homeY * SQUARE_SIDE_LENGTH,
              DIRECTION_EAST);
}

static void set_ascii_cell(char rows[DIMENSIONS][DIMENSIONS + 1],
                           int x, int y, char value) {
    rows[y][x] = value;
}

static int count_ground_truth_markers(int map[DIMENSIONS][DIMENSIONS]) {
    int total = 0;
    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            if (map[x][y] >= 3) {
                total += map[x][y] - 2;
            }
        }
    }
    return total;
}

static void run_marker_mission(char rows[DIMENSIONS][DIMENSIONS + 1]) {
    int map[DIMENSIONS][DIMENSIONS];
    Robot robot;
    Mission mission = {0};

    map_from_ascii(rows, map, &robot);
    int initialMarkerTotal = count_ground_truth_markers(map);
    CHECK(exploreAndReturn(&robot, map, &mission));
    CHECK(count_ground_truth_markers(map) == 0);
    CHECK(mission.delivered == initialMarkerTotal);
    CHECK(mission.energy == -mission.steps);
}

static void initialize_ascii_map(char rows[DIMENSIONS][DIMENSIONS + 1]) {
    for (int y = 0; y < DIMENSIONS; y++) {
        for (int x = 0; x < DIMENSIONS; x++) {
            rows[y][x] = '.';
        }
        rows[y][DIMENSIONS] = '\0';
    }
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

static void test_stacked_markers(void) {
    char rows[DIMENSIONS][DIMENSIONS + 1];
    initialize_ascii_map(rows);
    set_ascii_cell(rows, DIMENSIONS / 2, DIMENSIONS / 2, 'H');
    set_ascii_cell(rows, DIMENSIONS / 2 + 1, DIMENSIONS / 2, '3');

    watchdog_start(20);
    run_marker_mission(rows);
    watchdog_stop();
}

static void test_visible_marker_off_frontier_path(void) {
    char rows[DIMENSIONS][DIMENSIONS + 1];
    initialize_ascii_map(rows);
    set_ascii_cell(rows, DIMENSIONS / 2, DIMENSIONS / 2, 'H');
    set_ascii_cell(rows, DIMENSIONS / 2, DIMENSIONS / 2 - 1, '1');

    watchdog_start(20);
    run_marker_mission(rows);
    watchdog_stop();
}

int main(void) {
    test_smoke();
    test_observe_updates_belief_and_markers();
    test_stacked_markers();
    test_visible_marker_off_frontier_path();
    puts("tests passed");
    return EXIT_SUCCESS;
}
