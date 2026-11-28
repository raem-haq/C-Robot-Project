#include "dstar.h"
#include <assert.h>
#include <limits.h>
#include <stdlib.h>

static bool entry_is_current(const DStar *ds, const QueueEntry *entry);
static void discard_stale_entries(DStar *ds);
static int compare_queue_entries(const void *left, const void *right);
static int heuristic(Point a, Point b);
static bool in_bounds(Point p);

static int compare_queue_entries(const void *left, const void *right) {
    const QueueEntry *leftEntry = left;
    const QueueEntry *rightEntry = right;
    return key_compare(leftEntry->key, rightEntry->key);
}

void dstar_init(DStar *ds, Belief *robotBelief) {
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
    ds->map = robotBelief;
    heap_init(&ds->queue, DIMENSIONS * DIMENSIONS, sizeof(QueueEntry), compare_queue_entries);
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

Point queue_top_point(DStar *ds) {
    assert(ds != NULL);
    discard_stale_entries(ds);
    assert(!heap_empty(&ds->queue));

    const QueueEntry *entry = heap_peek(&ds->queue);
    return entry->point;
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

Key queue_top_key(DStar *ds) {
    assert(ds != NULL);
    discard_stale_entries(ds);

    if (heap_empty(&ds->queue)) {
        return (Key){INT_MAX, INT_MAX};
    }

    const QueueEntry *entry = heap_peek(&ds->queue);
    return entry->key;
}

Key calculate_key(DStar *ds, Point p) {
    int v = min(ds->g[p.x][p.y], ds->rhs[p.x][p.y]);
    return (Key){v + heuristic(ds->home, p), v};
}

static int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

void update_vertex(DStar *ds, Point p) {
    if (!is_same_point(p, ds->start)) {
        int smallest = INT_MAX;
        for (int i = 0; i < 4; i++) {
            Point pred = {p.x + directionX[i], p.y + directionY[i]};
            if (!in_bounds(pred))
                continue;
            ds->rhs[p.x][p.y] = min(smallest, ds->g[pred.x][pred.y] + edge_cost(ds, p, pred) );
        }
        ds->rhs[p.x][p.y] = smallest;
    }
    queue_remove(ds, p);
    if (ds->g[p.x][p.y] != ds->rhs[p.x][p.y]) {
        queue_insert(ds, p, calculate_key(ds, p));
    }
}

static bool is_same_point(Point a, Point b) {
    return (a.x == b.x && a.y == b.y);
}

static int directionX[] = {0, 1, 0, -1};
static int directionY[] = {-1, 0, 1, 0};

void compute_shortest_path(DStar *ds) {
    Point start = ds->start;
    while (key_compare(queue_top_key(ds), calculate_key(ds, ds->start)) < 0 ||
           ds->rhs[start.x][start.y] > ds->g[start.x][start.y]) {
        Point u = queue_top_point(ds);
        Key keyOld = queue_top_key(ds);
        Key keyNew = calculate_key(ds, u);
        ds->expansions++;

        if (key_compare(keyOld, keyNew) < 0) {
            queue_insert(ds, u, keyNew);
        } else if (ds->g[u.x][u.y] > ds->rhs[u.x][u.y]) {
            ds->g[u.x][u.y] = ds->rhs[u.x][u.y];
            queue_remove(ds, u);
            for (int i = 0; i < 4; i++) {
                Point pred = {u.x + directionX[i], u.y + directionY[i]};
                if (in_bounds(pred)) update_vertex(ds, pred);
            }
        } else {
            int g_old = ds->g[u.x][u.y];
            ds->g[u.x][u.y] = INT_MAX;
            update_vertex(ds, u);
            for (int i = 0; i < 4; i++) {
                Point pred = {u.x + directionX[i], u.y + directionY[i]};
                if (in_bounds(pred)) update_vertex(ds, pred);
            }
        }
    }
}

static bool in_bounds(Point p) {
    return (p.x >= 0 && p.x < DIMENSIONS && p.y >= 0 && p.y < DIMENSIONS);
}

/*
void dstar_main(DStar* ds){
    ds->last = ds->start;
    dstar_init(ds);
    compute_shortest_path(ds);
    while (!is_same_point(ds->start, ds->last)){

    }
}

static Point min_start(DStar *ds){
    Point argmin;
    int m;
    for (int i = 0; i < 4; i++){
        Point pred = {ds->start.x + directionX[i], ds->start.y + directionY[i]};


    }
}
*/
static bool are_adjacent(Point a, Point b){
    return abs(a.x - b.x) + abs(a.y - b.y) == 1;
}

static int edge_cost(DStar *ds, Point from, Point to) {
    assert(are_adjacent(from, to));
    assert(in_bounds(from));
    assert(in_bounds(to));
    return ds->map->cell[to.x][to.y] == BLOCKED ? INT_MAX : 1;
}