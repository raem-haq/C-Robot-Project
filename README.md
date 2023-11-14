# C-Robot-Project

This coursework completes all 6 stages. My program reads in coordinates for obstacles and markers from a text file, and these values were randomly generated coordinates, then inputted by me into the text file.

My traversal algorithm is based on DFS. I don't check the number of markers on the grid (or the number of markers remaining to see if the algorithm should stop), because I wanted the robot to have no information on the 

## running the program

To compile, you should write the following command:

```gcc -o courseWork main.c graphics.c robot.c grid.c```

now to execute the program, on windows, do the following,

```./courseWork | java -jar drawapp-3.0.jar```

or, you also have the option to input your own starting location and direction. For example, if you wanted to start from (9,8) facing east, you can do:

```./courseWork 9 8 east | java -jar drawapp-3.0.jar```

You must always put the direction in lowercase. The coordinate system is such that the top left corner is (0,0) and $y$ increases as you go down the screen and $x$ increases as you move right, $x, y$ must be non-negative integers. Also $x, y$ must be strictly less than ```dimensions``` in ```constants.h```.

## changing obstacle and marker positions

Marker and obstacle locations are read from text files, ```obstacles.txt``` and ```markers.txt```. If you want to modify these files, make sure you don't place a marker and an obstacle, a marker and the home square or an obstacle and the home square, on the same square, which will cause issues and display an error message.

## changing the animation speed

If you want to speed up or slow down the program, go to ```constants.h``` and decrease or increase the ```waitTimeTurn``` and/or ```waitTimeMove``` variables, respectively.
