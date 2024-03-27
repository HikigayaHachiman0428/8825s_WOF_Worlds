#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#undef __ARM_NEON__
#undef __ARM_NEON

#include "devices.h"
#include "vex.h"
#include "math.h"
#include "mathFunctions.h"

using namespace std;
using namespace Eigen;

// #define rot (getGyro + PI / 2.0)

typedef enum
{
    CIRCLE = 0x00,
    LINE = 0x01
} breakMode_;

double robotSpeed = 0;

class odometry
{
public:
    Vector2d mountingPos; /* describes how the odom system is mounted on the robot.
                        essentially how the tracking wheels' coordinate system deviates
                        from the robot's coordinate system.
                        center of rotation is (0, 0), front is y axis*/
    double mountingAngle;
    Vector2d trackingPos;
    Vector2d pos; // x, y
    double xrot, rot;
    Vector2d centering;
    Vector2d lastPos;
    Vector3d sensor;
    Vector3d lastSensor;

    Matrix2d propCoeffs;

    double alpha, beta;
    double verDeviDeg = 90, horDeviDeg = 0;
    Vector2d robotVelo;

    int veloUpdateCounter = 1;
    Vector2d veloUpdateP0, veloUpdatePt;

    double forceUpdate = 0;

public:
    odometry(Vector2d mountingPos_, double mountingAngle_, Vector2d initPos_, double initAngle_);
    void poseTracking();
    void setPose(Vector2d setPos, double theta);
    void setPose(double x, double y, double theta);
    void setPose(double x, double y);
    // void debug();
    void calcMove();
    void PID_Move();
    void calcTurn();
    void PID_Turn();
};

int trackingWrapper();

/////////////////////

odometry::odometry(Vector2d mountingPos_, double mountingAngle_, Vector2d initPos_, double initAngle_)
// : mountingPos(mountingPos_), mountingAngle(mountingAngle_), pos(initPos_), Drive_(_Drive_)
{
    cout << "debug" << Rotate(mountingPos_, initAngle_) << endl;
    trackingPos = initPos_ + Rotate(mountingPos_, initAngle_);
    IMU.setRotation(rad2deg(initAngle_), rotationUnits::deg);
    mountingPos = mountingPos_;
    mountingAngle = mountingAngle_;
}

void odometry::poseTracking()
{
    // std::chrono::time_point<std::chrono::high_resolution_clock> loopStart, loopEnd;
    // std::chrono::duration<double, std::milli> loopTime = std::chrono::milliseconds(1), elapsed;

    sensor << robotX, robotY, deg2rad(getGyro + mountingAngle);
    lastSensor = sensor;
    veloUpdateCounter = 1;
    while (1)
    {
        // loopStart = chrono::high_resolution_clock::now();
        /*executed code*/ //////////
        sensor << robotX, robotY, deg2rad(getGyro + mountingAngle);
        xrot = cstrAng(deg2rad(getGyro + mountingAngle));
        rot = cstrAng(deg2rad(getGyro + mountingAngle + 90));
        robotVelo << (sensor[0] - lastSensor[0]) * cos(horDeviDeg / 180 * M_PI) + (sensor[1] - lastSensor[1]) * sin(verDeviDeg / 180 * M_PI), (sensor[1] - lastSensor[1]) * cos(horDeviDeg / 180 * M_PI) + (sensor[0] - lastSensor[0]) * sin(verDeviDeg / 180 * M_PI);
        if (veloUpdateCounter <= 100)
            veloUpdateCounter++;
        else
        {
            veloUpdateCounter = 0;
            veloUpdateP0 = veloUpdatePt;
            veloUpdatePt = trackingPos;
            robotSpeed = (veloUpdatePt - veloUpdateP0).norm() / 5;
            // cout<<pos.transpose()<<endl;
        }
        delay(1);
        if (robotVelo.norm() == 0 && !forceUpdate)
            continue;
        else if (forceUpdate)
            forceUpdate = 0;
        alpha = lastSensor[2];
        beta = sensor[2];
        if (alpha != beta)
        {
            propCoeffs << (sin(beta) - sin(alpha)) / (beta - alpha), (cos(beta) - cos(alpha)) / (beta - alpha),
                (-cos(beta) + cos(alpha)) / (beta - alpha), (sin(beta) - sin(alpha)) / (beta - alpha);
        }
        else
        {
            propCoeffs = R(alpha);
        }
        trackingPos += propCoeffs * robotVelo;
        centering = Rotate(-(mountingPos), xrot);
        pos = trackingPos + centering;
        lastSensor = sensor;
        ///////////////////////////
        // loopEnd = chrono::high_resolution_clock::now();
        // this_thread::sleep_for(loopTime - (loopEnd - loopStart));
    }
}

void odometry::setPose(Vector2d setPos, double theta)
{
    trackingPos = setPos + Rotate(mountingPos, theta);
    IMU.setRotation(rad2deg(theta), rotationUnits::deg);
}

void odometry::setPose(double x, double y, double theta)
{
    Vector2d setPos(x, y);
    trackingPos = setPos + Rotate(mountingPos, theta);
    IMU.setRotation(rad2deg(theta), rotationUnits::deg);
}

void odometry::setPose(double x, double y)
{
    Vector2d setPos(x, y);
    trackingPos = setPos + Rotate(mountingPos, xrot);
    forceUpdate = 1;
}

Vector2d INITPOS = Vector2d(0, 0);
double INITANGLE = 0;

Vector2d MOUNTINGPOS = Vector2d(-6.5, -14.5); // center of rotation to tracking point
double MOUNTINGANGLE = 0;

odometry Odometry(MOUNTINGPOS, MOUNTINGANGLE, INITPOS, INITANGLE);

int trackingWrapper()
{
    Odometry.poseTracking();
    return 0;
}

#endif