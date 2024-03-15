#undef __ARM_NEON__
#undef __ARM_NEON

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "v5.h"
#include "v5_vcs.h"

#include <eigen-3.4.0/Eigen/Dense>


#define delay vexDelay

// #define waitUntil(condition)                                                   \
//   do {                                                                         \
//     wait(5, msec);                                                             \
//   } while (!(condition))

// #define repeat(iterations)                                                     \
//   for (int iterator = 0; iterator < iterations; iterator++)