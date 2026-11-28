#include "dstar.h"
#include <limits.h>
#include <assert.h>

void dstar_init(DStar *ds) {
    assert(ds != NULL);

    for (int x = 0; x < DIMENSIONS; x++) {
        for (int y = 0; y < DIMENSIONS; y++) {
            ds->g[x][y] = INT_MAX;
            ds->rhs[x][y] = INT_MAX;
            ds->inQueue[x][y] = false;
            ds->queuedKey[x][y] = (Key){INT_MAX, INT_MAX};
        }
    }

    ds->start = (Point){0, 0};
    ds->last = (Point){0, 0};
    ds->home = (Point){0, 0};
    ds->km = 0;
    ds->expansions = 0;
    heap_init(&ds->queue, DIMENSIONS * DIMENSIONS, sizeof(QueueEntry),
              compare_queue_entries);
}

static int compare_queue_entries(const void *left, const void *right) {
    const QueueEntry *leftEntry = left;
    const QueueEntry *rightEntry = right;
    return key_compare(leftEntry->key, rightEntry->key);
}

int key_compare(Key a, Key b) {
    if (a.k1 < b.k1) {
        return -1;
    }
    if (a.k1 > b.k1) {
        return 1;
    }
    if (a.k2 < b.k2) {
        return -1;
    }
    if (a.k2 > b.k2) {
        return 1;
    }
    return 0;
}