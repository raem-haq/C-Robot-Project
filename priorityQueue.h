#ifndef PRIORITY_QUEUE_H
#define MINHEAP_H

#include <stddef.h>

typedef int (*CompareFn)(const void *a, const void *b);

typedef struct {
    void *data;
    int size;
    int capacity;
    size_t elementSize;
    CompareFn compare;
} MinHeap;

void heap_init(MinHeap*, int);
void heap_push(MinHeap*, const void*);
void heap_pop(MinHeap*) ;
int heap_empty(MinHeap*);
void heap_free(MinHeap*);

#endif