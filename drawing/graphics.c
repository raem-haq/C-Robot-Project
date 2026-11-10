#include <SDL2/SDL.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "graphics.h"

#define PI 3.14159265358979323846

static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_Texture *backgroundLayer;
static SDL_Texture *foregroundLayer;
static SDL_Texture *activeLayer;
static SDL_Color currentColour = { 0, 0, 0, 255 };
static int canvasWidth = 800;
static int canvasHeight = 600;
static int running = 1;
static int redrawRequested;

static void checkSDL(int result, const char *operation) {
  if (result < 0) {
    fprintf(stderr, "ERROR: %s: %s\n", operation, SDL_GetError());
    exit(EXIT_FAILURE);
  }
}

static void destroyGraphics(void) {
  SDL_DestroyTexture(foregroundLayer);
  SDL_DestroyTexture(backgroundLayer);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  foregroundLayer = NULL;
  backgroundLayer = NULL;
  renderer = NULL;
  window = NULL;
  SDL_Quit();
}

static void initialiseSDL(void) {
  if (window != NULL) {
    return;
  }
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    fprintf(stderr, "ERROR: SDL initialization failed: %s\n", SDL_GetError());
    exit(EXIT_FAILURE);
  }
  window = SDL_CreateWindow("C Robot Project", SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED, canvasWidth, canvasHeight,
                            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if (window == NULL) {
    fprintf(stderr, "ERROR: Unable to create SDL window: %s\n", SDL_GetError());
    SDL_Quit();
    exit(EXIT_FAILURE);
  }
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  if (renderer == NULL) {
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  }
  if (renderer == NULL || !SDL_RenderTargetSupported(renderer)) {
    fprintf(stderr, "ERROR: Unable to create an SDL renderer with render targets: %s\n",
            SDL_GetError());
    destroyGraphics();
    exit(EXIT_FAILURE);
  }
  checkSDL(SDL_RenderSetLogicalSize(renderer, canvasWidth, canvasHeight),
           "Unable to set SDL logical window size");
  if (atexit(destroyGraphics) != 0) {
    fprintf(stderr, "ERROR: Unable to register SDL cleanup\n");
    destroyGraphics();
    exit(EXIT_FAILURE);
  }
}

static void createLayers(void) {
  SDL_SetRenderTarget(renderer, NULL);
  SDL_DestroyTexture(foregroundLayer);
  SDL_DestroyTexture(backgroundLayer);
  backgroundLayer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                      SDL_TEXTUREACCESS_TARGET,
                                      canvasWidth, canvasHeight);
  foregroundLayer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888,
                                      SDL_TEXTUREACCESS_TARGET,
                                      canvasWidth, canvasHeight);
  if (backgroundLayer == NULL || foregroundLayer == NULL) {
    fprintf(stderr, "ERROR: Unable to create SDL drawing layers: %s\n", SDL_GetError());
    exit(EXIT_FAILURE);
  }
  checkSDL(SDL_SetTextureBlendMode(backgroundLayer, SDL_BLENDMODE_BLEND),
           "Unable to configure background layer");
  checkSDL(SDL_SetTextureBlendMode(foregroundLayer, SDL_BLENDMODE_BLEND),
           "Unable to configure foreground layer");
  SDL_Texture *layers[] = { backgroundLayer, foregroundLayer };
  for (size_t i = 0; i < sizeof(layers) / sizeof(layers[0]); ++i) {
    checkSDL(SDL_SetRenderTarget(renderer, layers[i]), "Unable to select SDL drawing layer");
    checkSDL(SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0),
             "Unable to configure SDL drawing layer");
    checkSDL(SDL_RenderClear(renderer), "Unable to clear SDL drawing layer");
  }
  activeLayer = foregroundLayer;
  checkSDL(SDL_SetRenderTarget(renderer, activeLayer),
           "Unable to select SDL drawing layer");
}

static void pollEvents(void) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_QUIT ||
        (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
      running = 0;
    } else if (event.type == SDL_WINDOWEVENT &&
               (event.window.event == SDL_WINDOWEVENT_EXPOSED ||
                event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)) {
      redrawRequested = 1;
    }
  }
}

static void presentScene(void) {
  pollEvents();
  if (!running) {
    return;
  }
  redrawRequested = 0;
  checkSDL(SDL_SetRenderTarget(renderer, NULL), "Unable to present SDL drawing");
  checkSDL(SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255),
           "Unable to set SDL window background");
  checkSDL(SDL_RenderClear(renderer), "Unable to clear SDL window");
  checkSDL(SDL_RenderCopy(renderer, backgroundLayer, NULL, NULL),
           "Unable to draw SDL background layer");
  checkSDL(SDL_RenderCopy(renderer, foregroundLayer, NULL, NULL),
           "Unable to draw SDL foreground layer");
  SDL_RenderPresent(renderer);
  checkSDL(SDL_SetRenderTarget(renderer, activeLayer), "Unable to select SDL drawing layer");
}

static void prepareDrawing(void) {
  initialiseSDL();
  if (backgroundLayer == NULL || foregroundLayer == NULL) {
    createLayers();
  }
  if (!running) {
    exit(EXIT_SUCCESS);
  }
  checkSDL(SDL_SetRenderTarget(renderer, activeLayer), "Unable to select SDL drawing layer");
  checkSDL(SDL_SetRenderDrawColor(renderer, currentColour.r, currentColour.g,
                                  currentColour.b, currentColour.a),
           "Unable to set SDL drawing colour");
}

static int compareDoubles(const void *left, const void *right) {
  double a = *(const double *)left;
  double b = *(const double *)right;
  return (a > b) - (a < b);
}

void drawLine(int x1, int y1, int x2, int y2) {
  prepareDrawing();
  checkSDL(SDL_RenderDrawLine(renderer, x1, y1, x2, y2), "Unable to draw SDL line");
  presentScene();
}

void drawRect(int x, int y, int width, int height) {
  SDL_Rect rect = { x, y, width, height };
  prepareDrawing();
  checkSDL(SDL_RenderDrawRect(renderer, &rect), "Unable to draw SDL rectangle");
  presentScene();
}

void fillRect(int x, int y, int width, int height) {
  SDL_Rect rect = { x, y, width, height };
  prepareDrawing();
  checkSDL(SDL_RenderFillRect(renderer, &rect), "Unable to fill SDL rectangle");
  presentScene();
}

static int ovalHalfWidth(int width, int height, double yOffset) {
  double rx = (width - 1) / 2.0;
  double ry = (height - 1) / 2.0;
  if (ry == 0) {
    return 0;
  }
  double ratio = yOffset / ry;
  return (int)lround(rx * sqrt(fmax(0.0, 1.0 - ratio * ratio)));
}

void fillOval(int x, int y, int width, int height) {
  prepareDrawing();
  if (width <= 0 || height <= 0) {
    return;
  }
  double rx = (width - 1) / 2.0;
  double ry = (height - 1) / 2.0;
  for (int row = 0; row < height; ++row) {
    int halfWidth = ovalHalfWidth(width, height, row - ry);
    checkSDL(SDL_RenderDrawLine(renderer, (int)lround(x + rx - halfWidth), y + row,
                                (int)lround(x + rx + halfWidth), y + row),
             "Unable to fill SDL oval");
  }
  presentScene();
}

void drawOval(int x, int y, int width, int height) {
  prepareDrawing();
  if (width <= 0 || height <= 0) {
    return;
  }
  double rx = (width - 1) / 2.0;
  double ry = (height - 1) / 2.0;
  for (int row = 0; row < height; ++row) {
    int halfWidth = ovalHalfWidth(width, height, row - ry);
    checkSDL(SDL_RenderDrawPoint(renderer, (int)lround(x + rx - halfWidth), y + row),
             "Unable to draw SDL oval");
    checkSDL(SDL_RenderDrawPoint(renderer, (int)lround(x + rx + halfWidth), y + row),
             "Unable to draw SDL oval");
  }
  presentScene();
}

static void arcPoints(int x, int y, int width, int height,
                      int startAngle, int arcAngle, int filled) {
  if (width <= 0 || height <= 0 || arcAngle == 0) {
    return;
  }
  int segments = abs(arcAngle);
  if (segments > 360) {
    segments = 360;
  }
  if (filled) {
    int count = segments + 2;
    int *px = malloc((size_t)count * sizeof(*px));
    int *py = malloc((size_t)count * sizeof(*py));
    if (px == NULL || py == NULL) {
      free(px);
      free(py);
      fprintf(stderr, "ERROR: Unable to allocate SDL arc points\n");
      exit(EXIT_FAILURE);
    }
    px[0] = x + width / 2;
    py[0] = y + height / 2;
    for (int i = 0; i <= segments; ++i) {
      double angle = -(startAngle + (double)arcAngle * i / segments) * PI / 180.0;
      px[i + 1] = (int)lround(x + width / 2.0 + width / 2.0 * cos(angle));
      py[i + 1] = (int)lround(y + height / 2.0 + height / 2.0 * sin(angle));
    }
    fillPolygon(count, px, py);
    free(px);
    free(py);
    return;
  }
  prepareDrawing();
  double previousAngle = -startAngle * PI / 180.0;
  int previousX = (int)lround(x + width / 2.0 + width / 2.0 * cos(previousAngle));
  int previousY = (int)lround(y + height / 2.0 + height / 2.0 * sin(previousAngle));
  for (int i = 1; i <= segments; ++i) {
    double angle = -(startAngle + (double)arcAngle * i / segments) * PI / 180.0;
    int nextX = (int)lround(x + width / 2.0 + width / 2.0 * cos(angle));
    int nextY = (int)lround(y + height / 2.0 + height / 2.0 * sin(angle));
    checkSDL(SDL_RenderDrawLine(renderer, previousX, previousY, nextX, nextY),
             "Unable to draw SDL arc");
    previousX = nextX;
    previousY = nextY;
  }
  presentScene();
}

void drawArc(int x, int y, int width, int height, int startAngle, int arcAngle) {
  arcPoints(x, y, width, height, startAngle, arcAngle, 0);
}

void fillArc(int x, int y, int width, int height, int startAngle, int arcAngle) {
  arcPoints(x, y, width, height, startAngle, arcAngle, 1);
}

void drawPolygon(int count, int x[], int y[]) {
  if (count < 2 || x == NULL || y == NULL) {
    return;
  }
  prepareDrawing();
  for (int i = 0; i < count; ++i) {
    int next = (i + 1) % count;
    checkSDL(SDL_RenderDrawLine(renderer, x[i], y[i], x[next], y[next]),
             "Unable to draw SDL polygon");
  }
  presentScene();
}

void fillPolygon(int count, int x[], int y[]) {
  if (count < 3 || x == NULL || y == NULL) {
    return;
  }
  double *intersections = malloc((size_t)count * sizeof(*intersections));
  if (intersections == NULL) {
    fprintf(stderr, "ERROR: Unable to allocate SDL polygon intersections\n");
    exit(EXIT_FAILURE);
  }
  int minY = y[0];
  int maxY = y[0];
  for (int i = 1; i < count; ++i) {
    if (y[i] < minY) minY = y[i];
    if (y[i] > maxY) maxY = y[i];
  }
  prepareDrawing();
  for (int row = minY; row < maxY; ++row) {
    int found = 0;
    double scanY = row + 0.5;
    for (int i = 0; i < count; ++i) {
      int next = (i + 1) % count;
      if ((y[i] <= scanY && y[next] > scanY) ||
          (y[next] <= scanY && y[i] > scanY)) {
        intersections[found++] = x[i] +
            (scanY - y[i]) * (x[next] - x[i]) / (y[next] - y[i]);
      }
    }
    qsort(intersections, (size_t)found, sizeof(*intersections), compareDoubles);
    for (int i = 0; i + 1 < found; i += 2) {
      checkSDL(SDL_RenderDrawLine(renderer, (int)ceil(intersections[i]),
                                  row, (int)floor(intersections[i + 1]), row),
               "Unable to fill SDL polygon");
    }
  }
  free(intersections);
  presentScene();
}

static const unsigned char font[36][7] = {
  {14,17,19,21,25,17,14}, {4,12,4,4,4,4,14}, {14,17,1,2,4,8,31},
  {30,1,1,14,1,1,30}, {2,6,10,18,31,2,2}, {31,16,16,30,1,1,30},
  {6,8,16,30,17,17,14}, {31,1,2,4,8,8,8}, {14,17,17,14,17,17,14},
  {14,17,17,15,1,2,12},
  {14,17,17,31,17,17,17}, {30,17,17,30,17,17,30}, {14,17,16,16,16,17,14},
  {28,18,17,17,17,18,28}, {31,16,16,30,16,16,31}, {31,16,16,30,16,16,16},
  {14,17,16,23,17,17,15}, {17,17,17,31,17,17,17}, {14,4,4,4,4,4,14},
  {7,2,2,2,2,18,12}, {17,18,20,24,20,18,17}, {16,16,16,16,16,16,31},
  {17,27,21,21,17,17,17}, {17,25,21,19,17,17,17}, {14,17,17,17,17,17,14},
  {30,17,17,30,16,16,16}, {14,17,17,17,21,18,13}, {30,17,17,30,20,18,17},
  {15,16,16,14,1,1,30}, {31,4,4,4,4,4,4}, {17,17,17,17,17,17,14},
  {17,17,17,17,17,10,4}, {17,17,17,21,21,21,10}, {17,17,10,4,10,17,17},
  {17,17,10,4,4,4,4}, {31,1,2,4,8,16,31}
};

static unsigned char glyphRow(unsigned char character, int row) {
  if (character >= '0' && character <= '9') {
    return font[character - '0'][row];
  }
  if (character >= 'A' && character <= 'Z') {
    return font[10 + character - 'A'][row];
  }
  switch (character) {
    case '-': return row == 3 ? 14 : 0;
    case '.': return row == 6 ? 4 : 0;
    case ':': return row == 2 || row == 5 ? 4 : 0;
    case '!': return row < 5 ? 4 : (row == 6 ? 4 : 0);
    case '?': return (unsigned char[]){14,17,1,2,4,0,4}[row];
    default: return (unsigned char[]){14,17,1,2,4,0,4}[row];
  }
}

void drawString(char *text, int x, int y) {
  if (text == NULL) {
    return;
  }
  prepareDrawing();
  int originX = x;
  for (const unsigned char *ch = (const unsigned char *)text; *ch != '\0'; ++ch) {
    if (*ch == '\n') {
      x = originX;
      y += 16;
      continue;
    }
    unsigned char character = (unsigned char)toupper(*ch);
    for (int row = 0; row < 7; ++row) {
      unsigned char bits = glyphRow(character, row);
      for (int column = 0; column < 5; ++column) {
        if (bits & (1u << (4 - column))) {
          SDL_Rect pixel = { x + column * 2, y + row * 2, 2, 2 };
          checkSDL(SDL_RenderFillRect(renderer, &pixel), "Unable to draw SDL text");
        }
      }
    }
    x += 12;
  }
  presentScene();
}

void displayImage(char *fileName, int x, int y) {
  if (fileName == NULL) {
    fprintf(stderr, "ERROR: Image filename cannot be null\n");
    return;
  }
  SDL_Surface *surface = SDL_LoadBMP(fileName);
  if (surface == NULL) {
    fprintf(stderr, "ERROR: Unable to load BMP image '%s': %s\n", fileName, SDL_GetError());
    return;
  }
  prepareDrawing();
  SDL_Texture *texture = SDL_CreateTextureFromSurface(renderer, surface);
  if (texture == NULL) {
    fprintf(stderr, "ERROR: Unable to create SDL image texture: %s\n", SDL_GetError());
    SDL_FreeSurface(surface);
    return;
  }
  SDL_Rect destination = { x, y, surface->w, surface->h };
  checkSDL(SDL_RenderCopy(renderer, texture, NULL, &destination),
           "Unable to draw SDL image");
  SDL_DestroyTexture(texture);
  SDL_FreeSurface(surface);
  presentScene();
}

void setColour(colour c) {
  static const SDL_Color colours[] = {
    {0, 0, 0, 255}, {0, 0, 255, 255}, {0, 255, 255, 255},
    {64, 64, 64, 255}, {128, 128, 128, 255}, {0, 255, 0, 255},
    {192, 192, 192, 255}, {255, 0, 255, 255}, {255, 165, 0, 255},
    {255, 175, 175, 255}, {255, 0, 0, 255}, {255, 255, 255, 255},
    {255, 255, 0, 255}
  };
  if ((unsigned int)c >= sizeof(colours) / sizeof(colours[0])) {
    fprintf(stderr, "ERROR: Invalid SDL colour value: %d\n", (int)c);
    return;
  }
  currentColour = colours[c];
}

static Uint8 clampColour(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return (Uint8)value;
}

void setRGBColour(int red, int green, int blue) {
  currentColour = (SDL_Color){ clampColour(red), clampColour(green),
                               clampColour(blue), 255 };
}

void clear(void) {
  prepareDrawing();
  checkSDL(SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0), "Unable to set SDL clear colour");
  checkSDL(SDL_RenderClear(renderer), "Unable to clear SDL drawing layer");
  presentScene();
}

void setWindowSize(int width, int height) {
  if (width <= 0 || height <= 0) {
    fprintf(stderr, "ERROR: SDL window dimensions must be positive\n");
    return;
  }
  canvasWidth = width;
  canvasHeight = height;
  initialiseSDL();
  SDL_SetWindowSize(window, width, height);
  checkSDL(SDL_RenderSetLogicalSize(renderer, width, height),
           "Unable to set SDL logical window size");
  createLayers();
  presentScene();
}

void sleep(int milliseconds) {
  if (milliseconds <= 0) {
    return;
  }
  initialiseSDL();
  int remaining = milliseconds;
  while (remaining > 0 && running) {
    Uint32 delay = remaining > 10 ? 10u : (Uint32)remaining;
    SDL_Delay(delay);
    remaining -= (int)delay;
    pollEvents();
    if (redrawRequested) {
      presentScene();
    }
  }
  if (!running) {
    exit(EXIT_SUCCESS);
  }
}

void foreground(void) {
  initialiseSDL();
  if (backgroundLayer == NULL || foregroundLayer == NULL) {
    createLayers();
  }
  activeLayer = foregroundLayer;
}

void background(void) {
  initialiseSDL();
  if (backgroundLayer == NULL || foregroundLayer == NULL) {
    createLayers();
  }
  activeLayer = backgroundLayer;
}

void waitForClose(void) {
  if (window == NULL) {
    return;
  }
  while (running) {
    SDL_Event event;
    if (SDL_WaitEvent(&event) == 0) {
      fprintf(stderr, "ERROR: Unable to wait for SDL events: %s\n", SDL_GetError());
      return;
    }
    if (event.type == SDL_QUIT ||
        (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
      running = 0;
    } else if (event.type == SDL_WINDOWEVENT &&
               (event.window.event == SDL_WINDOWEVENT_EXPOSED ||
                event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)) {
      presentScene();
    }
  }
}