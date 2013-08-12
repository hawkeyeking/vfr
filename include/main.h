#ifndef MAIN_H
#define MAIN_H

#include <tf/transform_datatypes.h>

const static int armid_left = 0;
const static int armid_right = 1;

const static int GOLD_SERIAL = 24;
const static int GREEN_SERIAL = 37;

void publish_automove(btVector3, int);


#endif
