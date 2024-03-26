#ifndef _AUTON_H_
#define _AUTON_H_

#include "vex.h"
#include "functions.h"
#include "devices.h"

#include "bezier_curve.h"
#include "pure_pursuit.h"
#include "odometry.h"
#include "tasks.h"

using namespace std;
int auton = 0;
bool finishedAuto = 0;

bool debug = 0;

void skills_first_step()
{
    Odometry.setPose(-142, -123);
    int startTime = Brain.Timer;
    cout << "--------start--------" << endl;
    delay(20);
    cout << "start coordinate" << Odometry.pos.transpose() << endl;
    /* push left side two triball */
    {
        moveItk(-100);
        timerDrive(-40, 200);
        openfrontLeft;
        drive(80);
        delay(100);
        drive(80, 40);
        delay(100);
        closefrontLeft;
        Kcor = 4;
        drive(60, 60, 0, getGyro);
        while (Odometry.pos[1] < -30)
        {
            drive(100);
        }
    }
    /* matload & precata */
    {
        SPLINECOUNT = 1;
        WAYPOINTS[0] = wayPointMat{
            {Odometry.pos[0], Odometry.pos[1]},
            {-127.49, -122.09},
            {-110.08, -144.74},
            {-92.66, -148.22}};
        SPEEDS[0] = -1.15;
        CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
        pursuitInit();

        while (PurePursuitFlag)
        {
            if (currentSplineIndex == 0 && t_intersection >= 0.7)
                moveItk(-100);
            delay(10);
        }

        drive(-100);
        delay(300);
        drive(50);
        delay(40);
        drive(100, 5);
        delay(200);
        moveItk(0);
        LookAt(-15, 96, 0, 1, 0.2); // 15,96

        Kcor = 1;
        timerDrive(-17, 190, getGyro);
        Kcor = 0.2;
        drive(-4);
    }
    /* catapult */
    {
        if (debug)
        {
            delay(3000);
            // LookAt(85, 120, 180);
        }
        else
        {
            moveCata(90);
            LookAt(-12, 96, 0, 0, 0, 22500);
        }
    }
    /* clear front barrier */
    {
        moveCata(0);
        drive(75, 100); // back out of match loading zone
        delay(700);
        drive(0);

        TURN.kp = 2;
        LookAt(-110, -20, 0, 5, 5);
        TURN.kp = 2;
        moveItk(100);
        // moveItk(100);
        timerDrive(90, 250);
        delay(100);
        timerDrive(-20, 50);

        openLeft;
        WingFlag++;
        TURN.kp = 2.5;
        PIDTurn(97, 5, 5);
        TURN.kp = 2.0;
        SPLINECOUNT = 1;
        WAYPOINTS[0] = wayPointMat{
            {-60.00, -29.00},
            {-10.00, -29.00},
            {40.00, -28.00},
            {89.00, -28.00}};
        SPEEDS[0] = -1.7;
        CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT); // move to push triballs
        pursuitInit();
        while (PurePursuitFlag)
        {
            delay(10);
        }
        Kcor = 2;
        closeLeft;
        WingFlag++;
        timerDrive(100, 300);
        moveItk(100);

        Kcor = 1;
    }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

int genius_route()
{
    task quickRetract(closeWingHook);
    int startTime = Brain.Timer;
    skills_first_step();

    PIDTurn(45, 5, 3);
    SPLINECOUNT = 1;
    WAYPOINTS[0] = wayPointMat{
        {80.00, -75.00},
        {95.00, -104.00},
        {110.00, -114.00},
        {120.00, -117.00}};
    SPEEDS[0] = -1.35;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        delay(10);
    }

    // SPLINECOUNT = 1;
    // WAYPOINTS[0] = wayPointMat{
    //     {-20, -25},
    //     {-5, -50},
    //     {50, -85},
    //     {110, -100}}; //-96
    // SPEEDS[0] = -1.5;
    // CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    // pursuitInit();
    // while (PurePursuitFlag)
    // {
    //     delay(10);
    // }
    TURN.kp = 2.5;

    openLeft;
    WingFlag++;
    getCorner = 1;

    PIDTurn(150, 5, 5); // sharp point turn
    openRight;
    WingFlag++;
    timerDrive(-100, 50);

    // quickRetract.stop();

    TURN.kp = 2;

    SPLINECOUNT = 1; // 3//move to cross tunnel
    // WAYPOINTS[0] = wayPointMat{
    //     {90, -50},
    //     {90, -65},
    //     {100, -80},
    //     {120, -80}};
    // SPEEDS[0] = -1.2;
    // WAYPOINTS[0] = wayPointMat{
    //     {120, -100},
    //     {135, -80},
    //     {145, -50},
    //     {155, 0}};
    // SPEEDS[0] = -1.2;
    WAYPOINTS[0] = wayPointMat{
        {160.00, 0.00},
        {157.00, 118.00},
        {130.00, 136.00},
        {105.00, 141.00}};
    SPEEDS[0] = -1.5;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        // if (currentSplineIndex == 1 && t_intersection >= 0.12)
        // {
        //     SPEEDS[1] = -1.1;
        //     // openRight;
        // }
        if (Odometry.pos[1] > -70)
        {
            closeRight;
        }
        if (Odometry.pos[1] > 40 && Odometry.pos[1] < 100)
        {
            SPEEDS[0] = -1;
            // openRight;
        }
        if (Odometry.pos[1] > 125)
        {
            SPEEDS[0] = -1.3;
        }
        if (Odometry.pos[1] > 15)
        {
            openRight;
        }
        delay(10);
    }
    while (robotSpeed > 0.7)
    {
        drive(-100);
        delay(10);
    }
    // Kcor = 1;
    //  timerDrive(-100, 300);
    timerDrive(70, 150, -90);
    // Kcor = 0.2;
    LookAt(-10000, 175, 180, 5, 3, 500);
    timerDrive(60, 150);
    delay(150);
    timerDrive(-100, 500);
    Kcor = 0.2; // 1st side push
    // delay(100);
    //  closeRight;

    cout << "point 1: " << Odometry.pos.transpose() << endl;

    SPLINECOUNT = 1;
    WAYPOINTS[0] = wayPointMat{
        {68.00, 161.00},
        {87.00, 135.00},
        {110.00, 125.00},
        {110.00, 112.00}};
    SPEEDS[0] = 1.4;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        if (t_intersection >= 0.3)
        {
            closeRight;
            // openRight;
        }
        // if (t_intersection > 0.35) closeRight;
        delay(10);
    }
    TURN.kp = 2;
    PIDTurn(0 + 95, 5, 5);
    TURN.kp = 2.2;
    moveItk(100);
    // open
    // openfrontLeft;

    // delay(100);
    // // moveItk(-100);
    // Kcor = 2;
    // timerDrive(-53, 850, 0 + 90 + 180); // gather triballs 1000
    // Kcor = 1;
    // moveItk(100);
    // closeRight;

    SPLINECOUNT = 2;
    WAYPOINTS[0] = wayPointMat{
        {120.00, 75.00},
        {110.00, 75.00},
        {100.00, 45.00},
        {90.00, 35.00}};
    SPEEDS[0] = 1.4;
    WAYPOINTS[1] = wayPointMat{
        {55.00, 25.00},
        {50.00, 25.00},
        {45.00, 25.00},
        {39.00, 25.00}};
    SPEEDS[1] = 1.5;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        delay(10);
    }

    LookAt(0, 1000, 0, 5, 5);
    timerDrive(60, 600);
    timerDrive(100, 300);
    moveItk(-100);
    closefrontLeft;
    closefrontRight;

    // SPLINECOUNT = 1;
    // WAYPOINTS[0] = wayPointMat{
    //     {0, 60},  // 70, 60
    //     {0, 50},  // 75, 57
    //     {0, 40},  // 80, 53
    //     {0, 25}}; // 82, 50
    // SPEEDS[0] = -1.4;
    // CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    // pursuitInit();
    // while (PurePursuitFlag)
    // {
    //     delay(10);
    // }
    Kcor = 1;
    timerDrive(-100, 500, 0);
    // encoderDrive(-100,800,0);
    moveItk(100);
    LookAt(-120, 0, 0, 5, 5);
    SPLINECOUNT = 1;
    WAYPOINTS[0] = wayPointMat{
        {-30.00, 20.00},
        {-45.00, 22.00},
        {-67.00, 35.00},
        {-72.00, 40.00}};
    SPEEDS[0] = 1.4;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        delay(10);
    }

    LookAt(0, 90, 0, 5, 5);
    openfrontLeft;
    openfrontRight;
    moveItk(100);
    SPLINECOUNT = 1;
    WAYPOINTS[0] = wayPointMat{
        {-70.00, 50.00},
        {-65.00, 50.00},
        {-50.00, 65.00},
        {-45.00, 90.00}};
    SPEEDS[0] = 1.1;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        if (Odometry.pos[1] > 75)
        {
            SPEEDS[0] = 1.5;
        }
        delay(10);
    }
    timerDrive(100, 320);
    moveItk(-100);
    closefrontRight;
    closefrontLeft;

    timerDrive(-50, 100);
    LookAt(-20, 0, 180, 5, 5);
    drive(-85, -100);
    delay(350);
    // timerDrive(-100,500,0);
    Kcor = 0.2;

    moveItk(100);
    LookAt(120, -20, 0, 5, 5);
    SPLINECOUNT = 1;
    WAYPOINTS[0] = wayPointMat{
        {30.00, 20.00},
        {45.00, 22.00},
        {65.00, 30.00},
        {70.00, 40.00}};
    SPEEDS[0] = 1.5;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        delay(10);
    }

    LookAt(0, 90, 0, 5, 5);
    openfrontLeft;
    openfrontRight;
    moveItk(100);
    SPLINECOUNT = 1;
    WAYPOINTS[0] = wayPointMat{
        {60.00, 50.00},
        {55.00, 50.00},
        {45.00, 60.00},
        {35.00, 95.00}};
    SPEEDS[0] = 1.1;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        if (Odometry.pos[1] > 75)
        {
            SPEEDS[0] = 1.5;
        }
        delay(10);
    }
    timerDrive(100, 220);
    moveItk(-100);
    closefrontLeft;
    closefrontRight;
    timerDrive(-100, 300);
    openfrontLeft;
    openfrontRight;
    LookAt(-30, 120, 0, 5, 5);
    drive(100, 24);
    delay(450);
    timerDrive(100, 350);
    closefrontLeft;
    closefrontRight;
    timerDrive(-100, 100);

    LookAt(-60, 0, 180, 5, 5);
    timerDrive(-100, 220);

    moveItk(100);
    TURN.kp = 2.5;
    LookAt(-90, 120, 180, 5, 3);
    TURN.kp = 2;
    SPLINECOUNT = 1; // gather triballs again for last side push
    WAYPOINTS[0] = wayPointMat{
        {-67.00, 65.00},
        {-73.00, 75.00},
        {-80.00, 95.00},
        {-90.00, 110.00}};
    SPEEDS[0] = -1.4;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        if (currentSplineIndex == 0 && t_intersection >= 0.2)
            openRight;
        if (currentSplineIndex == 0 && t_intersection >= 0.6)
            SPEEDS[0] = -1;
        delay(10);
    }

    // timerDrive(-50, 100);
    TURN.kp = 2.5;
    LookAt(-145, 80, 0, 3, 3); //-145, 60
    TURN.kp = 2;
    closeLeft;
    closeRight;

    // moveItk(100);
    SPLINECOUNT = 2;
    WAYPOINTS[0] = wayPointMat{
        {-122.00, 115.00},
        {-137.00, 98.00},
        {-150.00, 75.00},
        {-162.00, 55.00}};
    SPEEDS[0] = 1.1;
    WAYPOINTS[1] = wayPointMat{
        {-156.00, 100.00},
        {-130.00, 112.00},
        {-115.00, 148.00},
        {-100.00, 150.00}};
    SPEEDS[1] = -1.1;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    currentSplineIndex = 0;
    pursuitInit();
    while (PurePursuitFlag)
    {
        if (currentSplineIndex == 1 && Odometry.pos[1] > 50)
            openLeft;
        delay(10);
    }
    int b = Brain.timer(msec);
    while (robotSpeed > 0.3 && Brain.timer(msec) - b <= 400)
    {
        drive(-100);
        delay(10);
    }
    moveItk(-100);
    // timerDrive(-100, 300);
    // Kcor = 1;
    timerDrive(70, 150);
    TURN.kp = 2;
    LookAt(10000, 155, 180, 5, 5, 500);
    timerDrive(70, 150);
    delay(200);
    moveItk(-100);
    timerDrive(-100, 700, 90);
    Kcor = 0.2;
    // timerDrive(50, 350);
    // timerDrive(-100, 450);
    closeLeft;
    hangL.set(1);
    hangR.set(1);
    SPLINECOUNT = 1;
    // catapult returns to start position
    WAYPOINTS[0] = wayPointMat{
        {-80.00, 160.00},
        {-120.00, 135.00},
        {-142.00, 120.00},
        {-152.00, -10.00}};
    SPEEDS[0] = 1.35;
    CURVE = CubicBezier(WAYPOINTS, SPEEDS, SPLINECOUNT);
    pursuitInit();
    while (PurePursuitFlag)
    {
        if (t_intersection >= 0.75)
        {
            SPEEDS[0] = 1.57;
            cout << "Full time ::" << Brain.timer(msec) - startTime << endl;
        }
        vexDelay(10);
        // cout << "Full time ::" << Brain.timer(msec) - startTime << endl;
    }

    cout << "Full time final::" << Brain.timer(msec) - startTime << endl;
    timerDrive(100, 1500);
    Controller.Screen.clearLine(4);
    Controller.Screen.setCursor(4, 1);
    Controller.Screen.print(Brain.timer(msec) - startTime);
    while (1)
    {
        delay(10);
    }

    return 0;
}

void innerFAST()
{
    moveItk(100);
    rightWing.set(1);
    encoderDrive(-100, wheelDeg2cm(-1930));
    PIDTurn(-315, 3, 0.5);
    rightWing.set(0);
    encoderDrive(50, wheelDeg2cm(200));
    delay(50);
    PIDDrive(wheelDeg2cm(-1850));
    PIDTurn(-370);
    // moveItk(-100)
    // leftWing.set(1);
    timerDrive(30, 120); // 30 50
    PIDDrift_new(42, 58, -450);
    leftWing.set(1);
    timerDrive(70, 300);
    moveItk(-100);
    leftWing.set(0);
    timerDrive(-30, 300);
    PIDTurn(-400);
    PIDDrive(wheelDeg2cm(-1350));
    PIDTurn(-270);
    moveItk(100);
    encoderDrive(50, wheelDeg2cm(500));
    delay(100);
    timerDrive(-30, 100);
    PIDTurn(-270);
    PIDDrive(wheelDeg2cm(-950));
    PIDTurn(180, 5);
    timerDrive(-60, 400);
    // PIDDrift_new(30, 70, -135);
    timerDrive(30, 100);
    PIDTurn(-45);
    rightWing.set(1);
    timerDrive(30, 600);
    drive(-10, 100);
    delay(150);
    drive(10, -100);
    delay(150);
    drive(0, 0);
    timerDrive(-40, 100);
    rightWing.set(0);
    timerDrive(-40, 200);
    leftWing.set(1);
    timerDrive(40, 800);

    PIDDrift_new(30, 70, 180, 5);
    timerDrive(100, 400);
    moveItk(-100);
    timerDrive(-50, 300);
    moveItk(100);
    timerDrive(100, 400);
    moveItk(-100);
    leftWing.set(0);
    timerDrive(-30, 200);

    PIDTurn(90);
    encoderDrive(100, wheelDeg2cm(2100));
    leftWing.set(1);
    drive(30, -30);
    delay(250);
    drive(0);
}

void outer_awp()
{
    cout << " ---- 15s Starting ---- " << endl;
    float startTime = Brain.Timer;
    timerDrive(-40, 200);
    moveItk(-100);
    timerDrive(30, 300);
    rightFrontWing.set(1);
    drive(-20, 100);
    delay(150);
    drive(20, -100);
    delay(150);
    drive(0, 0);
    rightFrontWing.set(0);
    PIDDrift_new(20, 80, -90);
    // leftFrontWing.set(1);
    encoderDrive(70, 1800);
    PIDDrive(-2100 * extGearRatio / wheelRadius);
    PIDTurn(180);
    encoderDrive(60, -600);
    PIDTurn(90);
    leftWing.set(1);
    encoderDrive(60, -2000);
    drive(-20, -20, 80, getGyro);
    cout << "endTime: " << Brain.Timer - startTime << endl;
}

void outer_alliance()
{
    cout << " ---- 15s Starting ---- " << endl;
    float startTime = Brain.Timer;
    timerDrive(-40, 200);
    moveItk(-100);
    timerDrive(30, 300);
    rightFrontWing.set(1);
    drive(-20, 100);
    delay(150);
    drive(20, -100);
    delay(150);
    drive(0, 0);
    encoderDrive(60, -1000);
    PIDTurn(-30);
    encoderDrive(60, 1000);
    PIDDrift_new(35, 65, 180);
    moveItk(100);
    encoderDrive(60, 1600);
    PIDDrive(-900 * extGearRatio / wheelRadius);
    PIDTurn(-90, 5, 5);
    leftFrontWing.set(1);
    drive(60);
    delay(200);
    moveItk(-100);
    delay(100);
    drive(0);
    leftFrontWing.set(0);
    timerDrive(-30, 100);
    PIDTurn(0, 5, 5);
    rightWing.set(1);
    timerDrive(-60, 700);
    drive(-20, -20, -160, getGyro);
    cout << "endTime: " << Brain.Timer - startTime << endl;
}

void innerElimination()
{
    cout << " ---- 15s Starting ---- " << endl;
    float startTime = Brain.Timer;
    task FLW(robFirstWing);
    FLWT = 146;
    moveItk(100);
    rightWing.set(1);
    encoderDrive(-100, -62);
    PIDTurn(-315, 3, 0.5);
    rightWing.set(0);
    Kcor = 3;
    encoderDrive(50, 9.4);
    // delay(50);
    timerDrive(-30, 100);
    PIDTurn(-315);
    PIDDrive(-94.86);
    PIDTurn(-358, 5);
    encoderDrive(-100, -110);
    timerDrive(-60, 300);
    timerDrive(30, 100);
    // leftWing.set(1);
    // PIDDrift_new(30,70,-90);
    PIDTurn(-405);
    // PIDTurn(-45);
    rightWing.set(1);
    // PIDDrive(3);
    // delay(200);
    timerDrive(30, 600);
    drive(-10, 100);
    delay(150);
    drive(10, -100);
    delay(150);
    drive(0, 0);
    timerDrive(-40, 100);
    rightWing.set(0);
    timerDrive(-40, 200);
    leftWing.set(1);
    timerDrive(40, 800);

    PIDDrift_new(30, 70, -360, 5);
    timerDrive(100, 400);
    moveItk(-100);
    timerDrive(-50, 300);
    moveItk(100);
    timerDrive(100, 400);
    moveItk(-100);
    leftWing.set(0);
    // timerDrive(-30, 200);

    /////
    PIDDrive(-49.35);
    PIDTurn(-289);
    moveItk(100);
    encoderDrive(70, 263.2);
    delay(50);
    PIDTurn(-375, 3, 0.5);
    leftFrontWing.set(1);
    rightFrontWing.set(1);
    PIDDrift_new(45, 55, -450);
    timerDrive(70, 600);
    timerDrive(-100, 300);
    ////
    cout << "endTime: " << Brain.Timer - startTime << endl;
}

#endif