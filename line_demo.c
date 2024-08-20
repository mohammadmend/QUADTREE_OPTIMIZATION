#include "./line_demo.h"

#include <time.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "vec.h"
#include <cilk/cilk.h>
#include "./graphic_stuff.h"
#include "./line.h"

#ifdef SERIAL
  #define cilk_for for
  #define cilk_spawn
  #define cilk_scope
#endif

static char* LineDemo_input_file_path;

void LineDemo_setInputFile(char* input_file_path) {
  LineDemo_input_file_path = input_file_path;
}

LineDemo* LineDemo_new() {
  LineDemo* lineDemo = malloc(sizeof(LineDemo));
  if (lineDemo == NULL) {
    return NULL;
  }

  lineDemo->count = 0;
  lineDemo->numFrames = 0;
  lineDemo->collisionWorld = NULL;
  return lineDemo;
}

void LineDemo_delete(LineDemo* lineDemo) {
  CollisionWorld_delete(lineDemo->collisionWorld);
  free(lineDemo);
}

// Read in lines from line.in and add them into collision world for simulation.
void LineDemo_createLines(LineDemo* lineDemo) {
  unsigned int lineId = 0;
  unsigned int numOfLines;
  window_dimension px1;
  window_dimension py1;
  window_dimension px2;
  window_dimension py2;
  window_dimension vx;
  window_dimension vy;
  int isGray;
  FILE *fin;
  fin = fopen(LineDemo_input_file_path, "r");
  if (fin == NULL) {
    fprintf(stderr, "Input file not found (%s)\n", LineDemo_input_file_path);
    exit(1);
  }

  fscanf(fin, "%d\n", &numOfLines);
  lineDemo->collisionWorld = CollisionWorld_new(numOfLines);

 while (EOF
      != fscanf(fin, "(%lf, %lf), (%lf, %lf), %lf, %lf, %d\n", &px1, &py1, &px2,
                &py2, &vx, &vy, &isGray)) {
    Line *line = malloc(sizeof(Line));

    // convert window coordinates to box coordinates
    windowToBox(&line->p1.x, &line->p1.y, px1, py1);
    windowToBox(&line->p2.x, &line->p2.y, px2, py2);

//line->length=lineLengthInBox(line->p1.x, line->p1.y,line->p2.x,line->p2.y);


line->length=Vec_length(Vec_subtract(line->p1, line->p2));
    // convert window velocity to box velocity
    velocityWindowToBox(&line->velocity.x, &line->velocity.y, vx, vy);
//line->length=Vec_length(Vec_subtract(line->p1, line->p2));
    // store color
    line->color = (Color) isGray;

    // store line ID
    line->id = lineId;
    lineId++;

    // transfer ownership of line to collisionWorld
    CollisionWorld_addLine(lineDemo->collisionWorld, line);
  }
  fclose(fin);

}

void LineDemo_setNumFrames(LineDemo* lineDemo, const unsigned int numFrames) {
  lineDemo->numFrames = numFrames;
}

void LineDemo_initLine(LineDemo* lineDemo) {
  LineDemo_createLines(lineDemo);
}

Line* LineDemo_getLine(LineDemo* lineDemo, const unsigned int index) {
  return CollisionWorld_getLine(lineDemo->collisionWorld, index);
}

unsigned int LineDemo_getNumOfLines(LineDemo* lineDemo) {
  return CollisionWorld_getNumOfLines(lineDemo->collisionWorld);
}

unsigned int LineDemo_getNumLineWallCollisions(LineDemo* lineDemo) {
  return CollisionWorld_getNumLineWallCollisions(lineDemo->collisionWorld);
}

unsigned int LineDemo_getNumLineLineCollisions(LineDemo* lineDemo) {
  return CollisionWorld_getNumLineLineCollisions(lineDemo->collisionWorld);
}

// The main simulation loop
bool LineDemo_update(LineDemo* lineDemo) {
  lineDemo->count++;
  CollisionWorld_updateLines(lineDemo->collisionWorld);
  if (lineDemo->count > lineDemo->numFrames) {
    return false;
  }
  return true;
}
