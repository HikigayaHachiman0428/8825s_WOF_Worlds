#ifndef _PPS_H_
#define _PPS_H_

#include "vex.h"
#include "bezier_curve.h"
#include "mathFunctions.h"
#include "functions.h"

#include "odometry.h"

// class PurePursuit
// {
// public:
//     CubicBezier curve;

// public:
//     PurePursuit();
//     void PursueCurve();
// };

// PurePursuit::PurePursuit()
// {

// }

double scanIntersection(Vector2d robotPos, CubicBezier *curve, int splineIndex, double ld)
{
    double t_intersection = 0;
    double currentMinErr = 9999999;
    double ldErr = 0;
    for (double t = 1; t >= 0; t -= 0.01)
    {
        ldErr = fabs(((curve->BezierPoint(t, splineIndex) - robotPos).norm()) - ld);
        if (((curve->BezierPoint(t, splineIndex) - robotPos).norm()) - ld < 0)
            return t;
        if (ldErr < currentMinErr)
        {
            t_intersection = t;
            currentMinErr = ldErr;
        }
    }
    return t_intersection;
}

// Vector2d scaleVelo(double vl, double vr)
// {
//     double diff = 0;
//     if (fabs(vl) > fabs(vr) && fabs(vl) > 100)
//     {
//         diff = sign(vl) * 100 - vl;
//         vl = sign(vl) * 100;
//         vr = vr + diff;
//     }
//     else if (fabs(vl) < fabs(vr) && fabs(vr) > 100)
//     {
//         diff = sign(vl) * 100 - vr;
//         vr = sign(vl) * 100;
//         vl = vl + diff;
//     }
//     return Vector2d(vl, vr);
// }

Vector2d scaleVelo(double linSpd, double angSpd)
{
    double vl = linSpd - angSpd;
    double vr = linSpd + angSpd;
    if (fabs(vr) > 100)
    {
        linSpd = sign(vr) * 100 - angSpd;
        vl = linSpd - angSpd;
        vr = sign(vr) * 100;
    }
    if (fabs(vl) > 100)
    {
        linSpd = sign(vl) * 100 - angSpd;
        vr = linSpd + angSpd;
        vl = sign(vl) * 100;
    }
    return Vector2d(vl, vr);
}

double t_intersection = 0;

int currentSplineIndex = 0;

bool PurePursuitFlag = 0;

bool softLaunchDrift = 1;

void PursueCurve(CubicBezier *curve)
{
    Vector2d robotPos(0.0, 0.0), targetPoint(0.0, 0.0);

    double currentMinErr = 9999999;

    double ld = 15; // look ahead distance (cm)
    double kld = 25;
    double ldErr = 0;
    double currentSpeed = 0;

    double rotErr = 0, linErr = 0, baseSpeed = 20, kSpd = 1.1, linSpeed = 0, krot = 20, angSpeed = 0; // kspd = 1

    currentSplineIndex = 0; // 为什么不清零？？？？？
    bool softLaunchFlag = 1;
    // double optiBaseSpeed = 0;
    double vl = 0, vr = 0;

    bool endflag = 0;

    int endTime = 0;
    double accele = 0;
    int printCounter = 0;
    double lastCurrentspeed = 0;
    Vector2d endDirVec = (curve->getWayPoint(3, curve->splineCount - 1) - curve->getWayPoint(2, curve->splineCount - 1));
    endDirVec.normalize();

    Vector2d currentDir(0.0, 1.0), baseDir(0.0, 1.0);
    float startTime = Brain.timer(msec);
    cout << "drift start" << Odometry.pos.transpose() << endl;
    while (PurePursuitFlag)
    {
        // update robot pos
        robotPos = Odometry.pos;
        ld = robotSpeed * kld;
        if (ld < 27)
            ld = 27;
        if (ld > 32)
            ld = 32;

        // ld = 30;
        // ld = 6;
        currentSpeed = curve->getSpeed(currentSplineIndex);

        if ((curve->getWayPoint(3, currentSplineIndex) - robotPos).norm() > ld)
            t_intersection = scanIntersection(robotPos, curve, currentSplineIndex, ld);
        else
            t_intersection = 1;

        if (!endflag)
        {
            targetPoint = curve->BezierPoint(t_intersection, currentSplineIndex);
        }
        else
        {
            targetPoint = curve->getWayPoint(3, curve->splineCount - 1) +
                          (endDirVec * (ld - (curve->getWayPoint(3, curve->splineCount - 1) - robotPos).norm()));
        }

        // cout<<targetPoint.transpose()<<" "<<ld<<" "<<rotErr<<" "<<t_intersection<<" "<<endflag<<endl;
        rotErr = cstrAng(vatan2(targetPoint - robotPos) - Odometry.rot + M_PI * (currentSpeed < 0));
        if (fabs(rotErr) >= M_PI_2)
            rotErr = sign(rotErr);
        // else
        //    rotErr = sin(rotErr);
        // wtf is this??

        // linV = kv * (45 - fabs(rotErr));
        // if (fabs(rotErr > 45))
        //     linV = 5;
        // linV = cap(linV * curve->getSpeed(currentSplineIndex), 60);

        linErr = 30 - fabs(rad2deg(rotErr));
        linErr = linErr < -49 ? -49 : linErr; //-45
        if (linErr > 4)
        {
            accele = accele + 0.02;
            if (accele <= linErr)
                linErr = accele;
        }
        else
            accele = 4;

        linErr = linErr > 20 ? 20 : linErr;
        // if(lastCurrentspeed * currentSpeed < 0) baseSpeed = 10;
        baseSpeed = baseSpeed < 48 ? baseSpeed + 1 : 48;
        lastCurrentspeed = currentSpeed;

        linSpeed = currentSpeed * (kSpd * linErr + baseSpeed);

        angSpeed = cap(fabs(currentSpeed) * sin(rotErr) * krot, 100);

        // vl = currentSpeed * linSpeed - fabs(currentSpeed) * rotErr * krot;
        // vr = currentSpeed * linSpeed + fabs(currentSpeed) * rotErr * krot;

        // if(softLaunchFlag && fabs(linV - optiLinV) > 3.0)
        // {
        //     optiLinV += 3 * sign(linV - optiLinV);
        //     linV = optiLinV;
        // }
        // if(fabs(linV - optiLinV) <= 3.0) softLaunchFlag = 0;

        driveVelo(scaleVelo(linSpeed, angSpeed));

        if (t_intersection == 1)
        {
            if (currentSplineIndex < curve->splineCount - 1)
            {
                t_intersection = 0.01;
                currentSplineIndex++;
                softLaunchFlag = 1;
                // baseSpeed = 10;
                if (curve->getSpeed(currentSplineIndex) * curve->getSpeed(currentSplineIndex - 1) < 0)
                    baseSpeed = 10;
            }
            else
                endflag = 1;
        }
        else
            endflag = 0;
        // currentDir = Rotate(baseDir, Odometry.xrot);
        // 这是啥啊？？？
        currentDir = curve->getWayPoint(3, curve->splineCount - 1) - robotPos;
        if (endflag)
            endTime++;
        else
            endTime = 0;
        if (endTime > 5 && robotSpeed <= 0.3)
            break;
        if (endflag && currentDir.dot(endDirVec) <= 0)
            break;

        printCounter++;
        if (printCounter == 40)
        {
            printCounter = 0;
            // cout << " target point: " << targetPoint.transpose() << " percent: " << t_intersection << " rotErr: " << rad2deg(rotErr) << " " << ld << endl;
            // cout << robotPos.transpose() << linSpeed << "  " << angSpeed << " time:" << Brain.timer(msec) << endl;
            // cout << "--------" << endl;
            cout << robotPos.transpose() << " time: " << Brain.timer(msec) - startTime << endl;
        }

        delay(5);
    }
    cout << "finished point " << robotPos.transpose() << " finished time: " << Brain.timer(msec) - startTime << endl;
    drive(0, 0);
}

void pursuitInit()
{
    currentSplineIndex = 0;
    t_intersection = 0;
    PurePursuitFlag = 1;
}

CubicBezier CURVE;
wayPointMat WAYPOINTS[10];
double SPEEDS[10];
int SPLINECOUNT = 0;
int PurePursuitControl()
{
    while (1)
    {
        if (PurePursuitFlag)
        {
            PursueCurve(&CURVE);
            PurePursuitFlag = 0;
        }
        delay(10);
    }
    return 0;
}

void updateWayPoints(int arrIndex, double x0, double y0, double x1, double y1,
                     double x2, double y2, double x3, double y3)
{
    WAYPOINTS[arrIndex] = wayPointMat{
        {x0, y0},
        {x1, y1},
        {x2, y2},
        {x3, y3}};
}
#endif