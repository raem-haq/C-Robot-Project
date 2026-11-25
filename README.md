# C Robot Project

C Robot Project is a grid-based robot exploration simulation written in C. The
robot navigates a map of obstacles and markers, rendered in an SDL2 window. It
builds its understanding of the map as it moves, collects markers, delivers
them to its starting square, and returns home after exploration is complete.

## How it works

The map is loaded from the files in `data/`, but route planning uses a separate
belief map. Initially, every cell is unknown. At each position, the robot
observes its current cell and nearby cells based on its heading, then chooses a
reachable frontier that offers new information relative to travel cost. A*
search finds routes through cells known to be free. When the robot finds a
marker, it returns to the home square to deliver it before continuing.

## Repository layout

- `main.c` — loads the map, initializes the robot, and starts the simulation.
- `simulation/` — grid setup, robot movement, and exploration planning.
- `drawing/` — SDL2 graphics adapter for the grid and simulation.
- `heap.c`, `heap.h` — min-heap used by route planning.
- `data/markers.txt`, `data/obstacles.txt` — marker and obstacle coordinates.
- `tests/` — headless test runner and graphics stubs.

## Requirements

- A C11-compatible compiler
- GNU Make
- `pkg-config`
- SDL2 development libraries

## Build and run

Build the graphical application from the repository root:

```sh
make
```

If `robot` is already up to date, Make prints `Nothing to be done for 'all'`.
To force a fresh build, remove the generated executables first:

```sh
make clean && make
```

Start at the default home square `(0, 0)`, facing east:

```sh
./robot
```

Optionally provide the home cell and initial direction:

```sh
./robot 9 8 east
```

Directions are `north`, `east`, `south`, or `west` (lowercase). Coordinates
refer to grid cells: `(0, 0)` is the top-left cell, x increases to the right,
and y increases downward. Both coordinates must be between `0` and
`DIMENSIONS - 1`. Close the SDL2 window with Escape or its close button.

Run the headless test runner with:

```sh
make test
```

The test build uses no SDL2 dependency; graphics calls are replaced by no-op
stubs. To remove the generated executables:

```sh
make clean
```

## Map data and configuration

Each line in `data/markers.txt` and `data/obstacles.txt` contains one cell
coordinate in `x,y` format, with no header or blank lines. Repeated marker
coordinates represent multiple markers in the same cell. Run the application
from the repository root so it can find these relative paths. Keep coordinates
within the configured grid, and do not place an obstacle on a marker or the
home square, or a marker on the home square.

`constants.h` sets the default grid size (`DIMENSIONS`, 10 cells per side), cell
size (`SQUARE_SIDE_LENGTH`, 50 pixels), and movement/turn delays. Override the
grid size for a build with Make's `DIM` variable:

```sh
make DIM=12
```

The `bench` target expects `bench/bench.c`; that source is not currently
included in the repository.

## To Do

