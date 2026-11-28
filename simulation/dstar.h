#ifndef DSTAR_H
#define DSTAR_H

#include <stdbool.h>

#include "../constants.h"
#include "../heap.h"
#include "belief.h"

typedef struct {
    int k1;
    int k2;
} Key;

typedef struct {
    Point point;
    Key key;
} QueueEntry;

typedef struct {
    int g[DIMENSIONS][DIMENSIONS];
    int rhs[DIMENSIONS][DIMENSIONS];
    bool inQueue[DIMENSIONS][DIMENSIONS];
    Key queuedKey[DIMENSIONS][DIMENSIONS];
    MinHeap queue;
    Point start;
    Point last;
    Point home;
    int km;
    unsigned long expansions;
} DStar;

void dstar_init(DStar *ds);
int key_compare(Key a, Key b);
void queue_insert(DStar *ds, Point p, Key key);
void queue_remove(DStar *ds, Point p);
void dstar_free(DStar *ds);

#endif
