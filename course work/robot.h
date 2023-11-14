typedef struct
{
    int xP;
    int yP;
    int direction;
    int isCarryingMarker;
} Robot;

void drawRobot(Robot *);

int canForward(Robot *, int **);
void left(Robot *, int**);
void right(Robot *, int**);
void forward(Robot *, int**);
void goHome(Robot *, char*, int, int**);
void turnAround(Robot *, int**);

int atMarker(Robot *, int **);
void initRobot(Robot *, int, int, int);
void drawForeground(Robot*, int**);