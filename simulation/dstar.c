#include "dstar.h"
#include <limits.h>
#include <assert.h>

static bool entry_is_current(const DStar *ds, const QueueEntry *entry);
static void discard_stale_entries(DStar *ds);
static int compare_queue_entries(const void *left, const void *right);

static int compare_queue_entries(const void *left, const void *right) {
    const QueueEntry *leftEntry = left;
    const QueueEntry *rightEntry = right;
    return key_compare(leftEntry->key, rightEntry->key);
}

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

void dstar_free(DStar *ds) {
    assert(ds != NULL);
    heap_free(&ds->queue);
}

void queue_insert(DStar *ds, Point p, Key key) {
    assert(ds != NULL);

    ds->inQueue[p.x][p.y] = true;
    ds->queuedKey[p.x][p.y] = key;
    QueueEntry entry = {p, key};
    heap_push(&ds->queue, &entry);
}

void queue_remove(DStar *ds, Point p) {
    assert(ds != NULL);

    ds->inQueue[p.x][p.y] = false;
}

static void discard_stale_entries(DStar *ds) {
    while (!heap_empty(&ds->queue)) {
        const QueueEntry *entry = heap_peek(&ds->queue);
        if (entry_is_current(ds, entry)) {
            return;
        }
        heap_pop(&ds->queue, NULL);
    }
}

static bool entry_is_current(const DStar *ds, const QueueEntry *entry) {
    Point p = entry->point;
    return ds->inQueue[p.x][p.y] && key_compare(entry->key, ds->queuedKey[p.x][p.y]) == 0;
}


Point queue_pop(DStar *ds) {
    assert(ds != NULL);
    discard_stale_entries(ds);
    assert(!heap_empty(&ds->queue));

    QueueEntry entry;
    heap_pop(&ds->queue, &entry);
    ds->inQueue[entry.point.x][entry.point.y] = false;
    return entry.point;
}

