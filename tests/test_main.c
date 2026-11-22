#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

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

int main(void) {
    test_smoke();
    puts("tests passed");
    return EXIT_SUCCESS;
}
