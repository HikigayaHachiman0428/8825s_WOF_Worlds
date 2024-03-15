#ifndef _MATHS_H_
#define _MATHS_H_

#include "vex.h"

#include "v5.h"
#include "v5_vcs.h"

#include <eigen-3.4.0/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define PI M_PI

#define cstrAng(theta) (remainder(theta, 2.0 * PI))
#define cstrAngDeg(theta) (remainder(theta, 360.0))

double vatan2(Vector2d input);

Matrix2d R(double theta);

Vector2d Rotate(Vector2d input, double theta);

Matrix2d R(double theta)
{
    Matrix2d ret;
    ret << cos(theta), -sin(theta), sin(theta), cos(theta);
    return ret;
}

Vector2d Rotate(Vector2d input, double theta)
{
    return R(theta) * input;
}


#define sign(x) (x > 0 ? 1 : (x == 0 ? 0 : -1))
#define deg2rad(theta) (((theta) / 180.0f) * M_PI)
#define rad2deg(theta) (((theta) * 180.0f) / M_PI)
#define cap(x, cap) (x > cap ? cap : (x < -cap ? -cap : x))

double vatan2(Vector2d input)
{
    if(input[0] != 0) return atan2(input[1], input[0]);
    else return M_PI_2 * sign(input[1]);
}

#endif