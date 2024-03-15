#ifndef _BEZIER_H_
#define _BEZIER_H_

#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>

using namespace Eigen;
using namespace std;

#define wayPointMat Matrix<double, 4, 2>

class CubicBezier
{
public:
    int splineCount;
    Matrix<double, 4, 2> *wayPoints;
    double *speed;

public:
    CubicBezier();
    CubicBezier(Matrix<double, 4, 2> *wayPoints_, double *speed_, int splineCount_);
    Vector2d getWayPoint(int pointIndex, int splineIndex);
    double getSpeed(int number);
    Vector2d BezierPoint(double t, int splineIndex);
    void BezierPrint();
};

CubicBezier::CubicBezier()
{
    wayPoints = nullptr;
    speed = nullptr;
    splineCount = 1;
}

CubicBezier::CubicBezier(Matrix<double, 4, 2> *wayPoints_, double *speed_, int splineCount_)
{
    wayPoints = wayPoints_;
    speed = speed_;
    splineCount = splineCount_;
}

Vector2d CubicBezier::getWayPoint(int pointIndex, int splineIndex)
{
    return (wayPoints[splineIndex](pointIndex, all)).transpose();
}

double CubicBezier::getSpeed(int speedIndex)
{
    return speed[speedIndex];
}

Vector2d CubicBezier::BezierPoint(double t, int splineIndex)
{
    RowVector4d coeffs(pow(1 - t, 3), 3 * pow(1 - t, 2) * t, 3 * (1 - t) * pow(t, 2), pow(t, 3));
    return (coeffs * wayPoints[splineIndex]);
}

void CubicBezier::BezierPrint()
{
    cout << "inside" << endl;
    for (int i = 0; i < splineCount; i = i + 1)
    {
        for (double t = 0; t <= 1; t = t + 0.01)
        {
            cout << BezierPoint(t, i).transpose() << endl;
            delay(10);
        }
    }
    cout<<"print end"<<endl;
}

#endif