#include "./collision_world.h"
#include <cilk/cilk.h>

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "./intersection_detection.h"
#include "./intersection_event_list.h"

//let's include the rect.h and quadtree.h for intersection
#include "Rectangle.h"
#include "QuadTree.h"

#ifdef SERIAL
  #define cilk_for for
  #define cilk_spawn
  #define cilk_scope
#endif


#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
void sort_eventlist(IntersectionEventNode *pNode);

CollisionWorld* CollisionWorld_new(const unsigned int capacity) {
  assert(capacity > 0);

  CollisionWorld* collisionWorld = malloc(sizeof(CollisionWorld));
  if (collisionWorld == NULL) {
    return NULL;
  }

  collisionWorld->numLineWallCollisions = 0;
  collisionWorld->numLineLineCollisions = 0;
  collisionWorld->timeStep = 0.5;
  collisionWorld->lines = malloc(capacity * sizeof(Line*));
  collisionWorld->numOfLines = 0;
  return collisionWorld;
}

void CollisionWorld_delete(CollisionWorld* collisionWorld) {

for (int i = 0; i < collisionWorld->numOfLines; i++) {
    free(collisionWorld->lines[i]);
  }
  free(collisionWorld->lines);
  free(collisionWorld);
}

unsigned int CollisionWorld_getNumOfLines(CollisionWorld* collisionWorld) {
  return collisionWorld->numOfLines;
}

void CollisionWorld_addLine(CollisionWorld* collisionWorld, Line *line) {
  collisionWorld->lines[collisionWorld->numOfLines] = line;
  collisionWorld->numOfLines++;
}

Line* CollisionWorld_getLine(CollisionWorld* collisionWorld,
                             const unsigned int index) {
  if (index >= collisionWorld->numOfLines) {
    return NULL;
  }
  return collisionWorld->lines[index];
}

void CollisionWorld_updateLines(CollisionWorld* collisionWorld) {
  //update the rectangles.
  update_rectangles(collisionWorld);
  CollisionWorld_detectIntersection(collisionWorld);
  CollisionWorld_updatePosition(collisionWorld);
  CollisionWorld_lineWallCollision(collisionWorld);
}

void CollisionWorld_updatePosition(CollisionWorld* collisionWorld) {
  double t = collisionWorld->timeStep;
for (int i = 0; i < collisionWorld->numOfLines; i++) {  
  Line *line = collisionWorld->lines[i];
    line->p1 = Vec_add(line->p1, Vec_multiply(line->velocity, t));
    line->p2 = Vec_add(line->p2, Vec_multiply(line->velocity, t));
  }
}
//This allows us to  calculate the bound of the area that we need making lineWallCollision more optimized
Bb calculateBoundBb(Line *line) {
  Bb bbox;
  bbox.minX = MIN(line->p1.x, line->p2.x);
  bbox.maxX = MAX(line->p1.x, line->p2.x);
  bbox.minY = MIN(line->p1.y, line->p2.y);
  bbox.maxY = MAX(line->p1.y, line->p2.y);
  return bbox;
}
bool intersectsWall(Bb bbox) {
  // using a predefined wall for bounding
  if (bbox.maxX > BOX_XMAX || bbox.minX < BOX_XMIN ||
      bbox.maxY > BOX_YMAX || bbox.minY < BOX_YMIN) {
    return true;
  }
  return false;
}


void CollisionWorld_lineWallCollision(CollisionWorld* collisionWorld) {
for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    bool collide = false;
        Bb bbox = calculateBoundBb(line);
    if(!intersectsWall(bbox)){
        continue;
    }else{
    // Right side
    if ((line->p1.x > BOX_XMAX || line->p2.x > BOX_XMAX)
        && (line->velocity.x > 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Left side
    if ((line->p1.x < BOX_XMIN || line->p2.x < BOX_XMIN)
        && (line->velocity.x < 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Top side
    if ((line->p1.y > BOX_YMAX || line->p2.y > BOX_YMAX)
        && (line->velocity.y > 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Bottom side
    if ((line->p1.y < BOX_YMIN || line->p2.y < BOX_YMIN)
        && (line->velocity.y < 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Update total number of collisions.
    if (collide == true) {
      collisionWorld->numLineWallCollisions++;
    }
    }
  }
}


void CollisionWorld_detectIntersection(CollisionWorld* collisionWorld) {
    IntersectionEventList intersectionEventList = IntersectionEventList_make();
    //detect collisions using our quad tree
   int collisions =quadtree_intersections(collisionWorld,&intersectionEventList);
    //update the collisions
    collisionWorld->numLineLineCollisions += collisions;
    //sort the event list
    IntersectionEventNode  * list = intersectionEventList.head;
    sort_event_list(list);

    //collision solver
    IntersectionEventNode * cNode = intersectionEventList.head;

    while(cNode != NULL){
        CollisionWorld_collisionSolver(collisionWorld,cNode->l1,cNode->l2,cNode->intersectionType);
        cNode = cNode->next;
    }

    IntersectionEventList_deleteNodes(&intersectionEventList);
}

unsigned int CollisionWorld_getNumLineWallCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineWallCollisions;
}

unsigned int CollisionWorld_getNumLineLineCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineLineCollisions;
}



void CollisionWorld_collisionSolver(CollisionWorld* collisionWorld,
                                    Line *l1, Line *l2,
                                    IntersectionType intersectionType) {
  assert(compareLines(l1, l2) < 0);
  assert(intersectionType == L1_WITH_L2
         || intersectionType == L2_WITH_L1
         || intersectionType == ALREADY_INTERSECTED);

  // Despite our efforts to determine whether lines will intersect ahead
  // of time (and to modify their velocities appropriately), our
  // simplified model can sometimes cause lines to intersect.  In such a
  // case, we compute velocities so that the two lines can get unstuck in
  // the fastest possible way, while still conserving momentum and kinetic
  // energy.
  if (intersectionType == ALREADY_INTERSECTED) {
    Vec p = getIntersectionPoint(l1->p1, l1->p2, l2->p1, l2->p2);

    if (Vec_length(Vec_subtract(l1->p1, p))
        < Vec_length(Vec_subtract(l1->p2, p))) {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p2, p)),
                                  Vec_length(l1->velocity));
    } else {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p1, p)),
                                  Vec_length(l1->velocity));
    }
    if (Vec_length(Vec_subtract(l2->p1, p))
        < Vec_length(Vec_subtract(l2->p2, p))) {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p2, p)),
                                  Vec_length(l2->velocity));
    } else {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p1, p)),
                                  Vec_length(l2->velocity));
    }
    return;
  }

  // Compute the collision face/normal vectors.
  Vec face;
  Vec normal;
  if (intersectionType == L1_WITH_L2) {
    Vec v = Vec_makeFromLine(*l2);
    face = Vec_normalize(v);
  } else {
    Vec v = Vec_makeFromLine(*l1);
    face = Vec_normalize(v);
  }
  normal = Vec_orthogonal(face);

  // Obtain each line's velocity components with respect to the collision
  // face/normal vectors.
  double v1Face = Vec_dotProduct(l1->velocity, face);
  double v2Face = Vec_dotProduct(l2->velocity, face);
  double v1Normal = Vec_dotProduct(l1->velocity, normal);
  double v2Normal = Vec_dotProduct(l2->velocity, normal);

  double newV1Normal = (((l1->length) - (l2->length)) / ((l1->length) + (l2->length))) * v1Normal
      + (2 * (l2->length) / ((l1->length) + (l2->length))) * v2Normal;
  double newV2Normal = (2 * (l1->length) / ((l1->length) + (l2->length))) * v1Normal
      + (((l2->length) - (l1->length)) / ((l2->length) + (l1->length))) * v2Normal;

  // Combine the resulting velocities.
  l1->velocity = Vec_add(Vec_multiply(normal, newV1Normal),
                         Vec_multiply(face, v1Face));
  l2->velocity = Vec_add(Vec_multiply(normal, newV2Normal),
                         Vec_multiply(face, v2Face));

  return;
}
