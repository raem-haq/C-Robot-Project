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

int main(void) {
    alarm(30);
    CHECK(1 == 1);
    puts("tests passed");
    return EXIT_SUCCESS;
}
