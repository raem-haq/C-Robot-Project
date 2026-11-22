CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra
DIM ?= 10

CPPFLAGS += -DDIMENSIONS=$(DIM)
LDLIBS += -lm

SIMULATION_SOURCES := $(wildcard simulation/*.c)

.PHONY: all test bench clean

all: robot

robot: main.c drawing/graphics.c $(SIMULATION_SOURCES) heap.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(shell pkg-config --cflags --libs sdl2) $(LDLIBS) -o $@

test_runner: tests/test_main.c $(SIMULATION_SOURCES) heap.c tests/graphics_stub.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDLIBS) -o $@

test: test_runner
	./test_runner

bench_runner: bench/bench.c $(SIMULATION_SOURCES) heap.c tests/graphics_stub.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDLIBS) -o $@

bench: bench_runner

clean:
	rm -f robot test_runner bench_runner
