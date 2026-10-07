#ifndef MINHEAP_H
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

void heap_init(MinHeap *heap, int capacity, size_t elementSize, CompareFn compare);
void heap_push(MinHeap*, const void*);
void heap_pop(MinHeap*, void *result);
int heap_empty(MinHeap*);
void heap_free(MinHeap*);

#endif