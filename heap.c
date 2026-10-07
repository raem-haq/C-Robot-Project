#include "priorityQueue.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void heap_error(MinHeap *heap, const char *message) {
    heap_free(heap);
    fprintf(stderr, "ERROR: %s\n", message);
    exit(EXIT_FAILURE);
}

static void *heap_element(MinHeap *heap, int index) {
    return (char *)heap->data + (size_t)index * heap->elementSize;
}

void heap_init(MinHeap *heap, int capacity) {
    heap->data = NULL;
    heap->size = 0;
    heap->capacity = 0;

    if (heap->elementSize == 0 || heap->compare == NULL || capacity < 0) {
        heap_error(heap, "Heap requires an element size, comparator, and nonnegative capacity");
    }

    if (capacity == 0) {
        return;
    }
    if ((size_t)capacity > SIZE_MAX / heap->elementSize) {
        heap_error(heap, "Heap capacity is too large");
    }

    heap->data = malloc((size_t)capacity * heap->elementSize);
    if (heap->data == NULL) {
        heap_error(heap, "Could not allocate heap storage");
    }
    heap->capacity = capacity;
}

void heap_push(MinHeap *heap, const void *element) {
    if (heap->size == INT_MAX) {
        heap_error(heap, "Heap is at maximum capacity");
    }

    void *elementCopy = malloc(heap->elementSize);
    if (elementCopy == NULL) {
        heap_error(heap, "Could not copy heap element");
    }
    memcpy(elementCopy, element, heap->elementSize);

    if (heap->size == heap->capacity) {
        int newCapacity = heap->capacity == 0 ? 4 : heap->capacity;
        if (heap->capacity != 0) {
            if (heap->capacity > INT_MAX / 2) {
                free(elementCopy);
                heap_error(heap, "Heap is at maximum capacity");
            }
            newCapacity = heap->capacity * 2;
        }
        if ((size_t)newCapacity > SIZE_MAX / heap->elementSize) {
            free(elementCopy);
            heap_error(heap, "Heap capacity is too large");
        }

        void *newData = realloc(heap->data, (size_t)newCapacity * heap->elementSize);
        if (newData == NULL) {
            free(elementCopy);
            heap_error(heap, "Could not grow heap storage");
        }
        heap->data = newData;
        heap->capacity = newCapacity;
    }

    int index = heap->size++;
    while (index > 0) {
        int parent = (index - 1) / 2;
        void *parentElement = heap_element(heap, parent);
        if (heap->compare(elementCopy, parentElement) >= 0) {
            break;
        }
        memcpy(heap_element(heap, index), parentElement, heap->elementSize);
        index = parent;
    }
    memcpy(heap_element(heap, index), elementCopy, heap->elementSize);
    free(elementCopy);
}

void heap_pop(MinHeap *heap) {
    if (heap->size == 0) {
        heap_error(heap, "Cannot pop from an empty heap");
    }
    if (heap->size == 1) {
        heap->size = 0;
        return;
    }

    --heap->size;

    int index = 0;
    while (2 * index + 1 < heap->size) {
        int left = 2 * index + 1;
        int right = left + 1;
        int smallest = left;

        if (right < heap->size &&
            heap->compare(heap_element(heap, right), heap_element(heap, left)) < 0) {
            smallest = right;
        }
        if (heap->compare(heap_element(heap, heap->size),
                          heap_element(heap, smallest)) <= 0) {
            break;
        }

        memcpy(heap_element(heap, index), heap_element(heap, smallest), heap->elementSize);
        index = smallest;
    }

    memcpy(heap_element(heap, index), heap_element(heap, heap->size), heap->elementSize);
}

int heap_empty(MinHeap *heap) {
    return heap->size == 0;
}

void heap_free(MinHeap *heap) {
    free(heap->data);
    heap->data = NULL;
    heap->size = 0;
    heap->capacity = 0;
}