# C-Robot-Project

This coursework completes all 6 stages. My program reads in coordinates for obstacles and markers from a text file, and these values were randomly generated coordinates, then inputted by me into the text file.

My traversal algorithm is based on DFS. I don't check the number of markers on the grid (or the number of markers remaining to see if the algorithm should stop), because I wanted the robot to have no information on the 

## running the program

Install the SDL2 development package and `pkg-config`, then compile from the
project directory:

```sh
gcc -Wall -Wextra -o courseWork main.c drawing/graphics.c simulation/robot.c \
  simulation/grid.c heap.c $(pkg-config --cflags --libs sdl2) -lm
```

Run it directly; the SDL2 window closes when you press Escape or its close
button. The graphics adapter includes a small bitmap font and loads BMP images
through SDL2:

```sh
./courseWork
```

You can also choose a starting location and direction. For example, to start
from (9,8) facing east:

```sh
./courseWork 9 8 east
```

You must always put the direction in lowercase. The coordinate system is such that the top left corner is (0,0) and $y$ increases as you go down the screen and $x$ increases as you move right, $x, y$ must be non-negative integers. Also $x, y$ must be strictly less than ```DIMENSIONS``` in ```constants.h```.

## changing obstacle and marker positions

Marker and obstacle locations are read from text files, ```data/obstacles.txt``` and ```data/markers.txt```. If you want to modify these files, make sure you don't place a marker and an obstacle, a marker and the home square or an obstacle and the home square, on the same square, which will cause issues and display an error message.

## changing the animation speed

If you want to speed up or slow down the program, go to ```constants.h``` and decrease or increase the ```WAIT_TIME_TURN``` and/or ```WAIT_TIME_MOVE``` variables, respectively.
