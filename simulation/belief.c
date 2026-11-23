#include "belief.h"

void belief_init(Belief *belief) {
    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            belief->cell[x][y] = UNKNOWN;
            belief->markers[x][y] = 0;
        }
    }
}
