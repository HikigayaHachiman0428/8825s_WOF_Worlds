#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include "vex.h"

#include "mathFunctions.h"

#include "devices.h"
#include "odometry.h"

using namespace vex;
using namespace std;

#define moveCata(v) \
    Cata.spin(vex::directionType::fwd, v * 120, vex::voltageUnits::mV)

#define moveItk(v) \
    Itk.spin(vex::directionType::fwd, v * 120, vex::voltageUnits::mV)

#define BA Controller.ButtonA
#define BX Controller.ButtonX
#define BB Controller.ButtonB
#define BY Controller.ButtonY
#define BR Controller.ButtonRight
#define BU Controller.ButtonUp
#define BL Controller.ButtonLeft
#define BD Controller.ButtonDown

#define R1 Controller.ButtonR1
#define R2 Controller.ButtonR2

#define L1 Controller.ButtonL1
#define L2 Controller.ButtonL2

#define JFB Controller.Axis3.position()
#define JLR Controller.Axis1.position()

#define delay(t) vexDelay(t)

#define openWings Wing.set(1)
#define closeWings Wing.set(0)

#define openLeft leftWing.set(1)
#define closeLeft leftWing.set(0)
#define openRight rightWing.set(1)
#define closeRight rightWing.set(0)

#define openfrontLeft leftFrontWing.set(1)
#define closefrontLeft leftFrontWing.set(0)
#define openfrontRight rightFrontWing.set(1)
#define closefrontRight rightFrontWing.set(0)

extern double wheelRadius = 5.08; // cm
extern double extGearRatio = 1.75;
extern double driveWidth = 31;        // cm
extern double maxSideFriction = 1650; // Newtons * 10^-2
extern double robotMass = 7;          // kg
double FLWT = 0;
#define wheelDeg2cm(theta) ((deg2rad(theta) / extGearRatio) * wheelRadius)

auto getWheelDis = [](motor Motor) -> double
{
    return wheelDeg2cm(Motor.position(deg));
};

auto getWheelSpd = [](motor Motor) -> double
{
    return wheelDeg2cm(Motor.velocity(vex::velocityUnits::dps));
};

#define driveEnc ((getWheelDis(LB) + getWheelDis(RB)) / 2)

#define driveSpd ((getWheelSpd(LB) + getWheelSpd(RB)) / 2)

auto absAng = [](float ang) -> float
{
    return ang - 2 * M_PI * floor((ang + M_PI) / (2 * M_PI));
};

auto absAngDeg = [](float ang) -> float
{
    return ang - 2 * 180 * floor((ang + 180) / (2 * 180));
};

void driveLeft(float voltage)
{
    LF.spin(vex::directionType::fwd, voltage * 120, vex::voltageUnits::mV);
    LM.spin(vex::directionType::fwd, voltage * 120, vex::voltageUnits::mV);
    LB.spin(vex::directionType::fwd, voltage * 120, vex::voltageUnits::mV);
}

void driveRight(float voltage)
{
    RF.spin(vex::directionType::fwd, voltage * 120, vex::voltageUnits::mV);
    RM.spin(vex::directionType::fwd, voltage * 120, vex::voltageUnits::mV);
    RB.spin(vex::directionType::fwd, voltage * 120, vex::voltageUnits::mV);
}

void drive(double v)
{
    driveLeft(v);
    driveRight(v);
}

void drive(double Vl, double Vr)
{
    driveLeft(Vl);
    driveRight(Vr);
}

extern double Kcor = 0.2;

void drive(double Vl, double Vr, double tarAng, double currentAng)
{
    driveLeft(Vl - absAngDeg(tarAng - currentAng) * Kcor);
    driveRight(Vr + absAngDeg(tarAng - currentAng) * Kcor);
}

void driveVelo_Cor(double Vl, double Vr, double tarAng, double currentAng)
{
    LF.spin(vex::directionType::fwd, Vl * 6 - absAngDeg(tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    LM.spin(vex::directionType::fwd, Vl * 6 - absAngDeg(tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    LB.spin(vex::directionType::fwd, Vl * 6 - absAngDeg(tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    RF.spin(vex::directionType::fwd, Vr * 6 + absAngDeg(tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    RM.spin(vex::directionType::fwd, Vr * 6 + absAngDeg(tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    RB.spin(vex::directionType::fwd, Vr * 6 + absAngDeg(tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
}

void driveVelo_Cor(double V, double tarAng, double currentAng)
{
    LF.spin(vex::directionType::fwd, V * 6 - (tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    LM.spin(vex::directionType::fwd, V * 6 - (tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    LB.spin(vex::directionType::fwd, V * 6 - (tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    RF.spin(vex::directionType::fwd, V * 6 + (tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    RM.spin(vex::directionType::fwd, V * 6 + (tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
    RB.spin(vex::directionType::fwd, V * 6 + (tarAng - currentAng) * Kcor, vex::velocityUnits::rpm);
}

void driveVelo(double vl, double vr)
{
    LF.spin(vex::directionType::fwd, vl * 6, vex::velocityUnits::rpm);
    LM.spin(vex::directionType::fwd, vl * 6, vex::velocityUnits::rpm);
    LB.spin(vex::directionType::fwd, vl * 6, vex::velocityUnits::rpm);
    RF.spin(vex::directionType::fwd, vr * 6, vex::velocityUnits::rpm);
    RM.spin(vex::directionType::fwd, vr * 6, vex::velocityUnits::rpm);
    RB.spin(vex::directionType::fwd, vr * 6, vex::velocityUnits::rpm);
}

void driveVelo(Vector2d velo)
{
    LF.spin(vex::directionType::fwd, velo[0] * 6, vex::velocityUnits::rpm);
    LM.spin(vex::directionType::fwd, velo[0] * 6, vex::velocityUnits::rpm);
    LB.spin(vex::directionType::fwd, velo[0] * 6, vex::velocityUnits::rpm);
    RF.spin(vex::directionType::fwd, velo[1] * 6, vex::velocityUnits::rpm);
    RM.spin(vex::directionType::fwd, velo[1] * 6, vex::velocityUnits::rpm);
    RB.spin(vex::directionType::fwd, velo[1] * 6, vex::velocityUnits::rpm);
}

// void gyroDrive(double Power, double gyrotarget, bool correct=1)
// {
//   moveMotor(chassisLB, Power - correct*1.2 * (ROTATION - gyrotarget));
//   moveMotor(chassisLF, Power - correct*1.2 * (ROTATION - gyrotarget));
//   moveMotor(chassisRB, Power + correct*1.2 * (ROTATION - gyrotarget));
//   moveMotor(chassisRF, Power + correct*1.2 * (ROTATION - gyrotarget));
// }

double leftLockTar = 0, rightLockTar = 0;
bool driveLock = 0;

int lockDrive()
{
    bool lastDriveLock = 0;
    double leftErr = 0, rightErr = 0;
    while (1)
    {
        if (driveLock)
        {
            if (!lastDriveLock)
            {
                leftLockTar = getWheelDis(LB);
                rightLockTar = getWheelDis(RB);
            }
            leftErr = leftLockTar - getWheelDis(LB);
            rightErr = rightLockTar - getWheelDis(RB);
            drive(leftErr * 3, rightErr * 3);
        }
        lastDriveLock = driveLock;
        vexDelay(5);
    }
    return 0;
}

struct PIDtype
{
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double error = 0;
    double integral = 0;
    double iStart = 0;
    double iCap = 30;
    double derivative = 0;
    double lastError = 0;
    double output = 0;
    double optimalOutput = 0;
    bool insideTolerance = 0;
    bool softLaunch = 0;
    bool launching = 0;
    bool resetSensor = 1;
    bool gyroCorrect = 1;
};

PIDtype DRIVE;
PIDtype DRIFT;
PIDtype TURN;

void init()
{
    hangL.set(0);
    hangR.set(0);
    IMU.setRate(5);
    DRIVE.kp = 6;
    DRIVE.ki = 0.1;
    DRIVE.kd = 45;
    DRIVE.iStart = 5;
    DRIVE.iCap = 5;
    DRIVE.softLaunch = 0;
    DRIFT.kp = 2.15;
    DRIFT.ki = 0;
    DRIFT.kd = 65;
    DRIFT.softLaunch = 1;
    TURN.kp = 2.2; // 4.5;
    TURN.kd = 16.5;  // 41;
    TURN.ki = 0.3; // 0.4;
    TURN.iStart = 8;
    TURN.iCap = 30;
    TURN.softLaunch = 0;
}

void PIDCalc(PIDtype *Struct, double error, double tolerance)
{
    Struct->error = error;
    Struct->insideTolerance = (fabs(error) <= tolerance);
    // if(Struct->insideTolerance&&fabs(Struct->lastError)>tolerance) resetTimer_(3);
    Struct->integral = (fabs(error) < Struct->iStart && error * Struct->lastError > 0) ? Struct->integral + Struct->error : 0;
    Struct->derivative = Struct->error - Struct->lastError;
    Struct->lastError = Struct->error;
    Struct->output = Struct->kp * Struct->error + Struct->ki * Struct->integral + Struct->kd * Struct->derivative;
    // Struct->output = Struct->output > 100 ? 100 : Struct->output;
    Struct->output = cap(Struct->output, 100);
    if (fabs(Struct->optimalOutput - Struct->output) < 5)
        Struct->launching = 0;
    if (Struct->softLaunch && fabs(Struct->optimalOutput - Struct->output) > 5 && Struct->launching)
    {
        Struct->optimalOutput = Struct->optimalOutput + sign(Struct->output - Struct->optimalOutput) * 2;
        Struct->output = Struct->optimalOutput;
    }
}

typedef enum driftDir
{
    lf = 0x00,
    lb = 0x01,
    rf = 0x02,
    rb = 0x03,
};

void PIDDrift(double arcLength, double targetAng, int stopTime = 20000, double tolerance = 1)
{
    // int lr = 1, fb = 1;
    // switch (Dir)
    // {
    // case lf:
    // {
    //     fb = 1;
    //     lr = 1;
    // }
    // case lb:
    // {
    //     fb = -1;
    //     lr = 1;
    // }
    // case rf:
    // {
    //     fb = 1;
    //     lr = -1;
    // }
    // case rb:
    // {
    //     fb = -1;
    //     lr = -1;
    // }
    // }
    double R = fabs(arcLength / absAng(deg2rad(targetAng - getGyro)));
    PIDtype *Struct = &DRIFT;
    Struct->insideTolerance = 0;
    double Vl = 0, Vr = 0;
    double gyroOffset = 90 - getGyro;
    double dGyro = getGyro + gyroOffset;
    double tarAng = M_PI_2;
    double maxV = sqrt(R * maxSideFriction / robotMass) * extGearRatio / wheelRadius;
    LB.resetPosition();
    RB.resetPosition();
    int startTime = Brain.Timer;
    cout << "drift start  " << Struct->kp << endl;
    Struct->optimalOutput = 30 * sign(arcLength);
    while ((!Struct->insideTolerance || Struct->derivative > 5) && Brain.Timer - startTime < stopTime) //||time_[3]<400)
    {
        dGyro = getGyro + gyroOffset;
        // Struct->launching = (fabs(driveEnc) < fabs(arcLength / 2));
        PIDCalc(Struct, arcLength - driveEnc, tolerance);
        if (fabs(driveEnc) > fabs(arcLength / 2))
            Struct->launching = 0;
        Struct->output = cap(Struct->output, maxV);
        Vr = (Struct->output) * ((R + sign(arcLength) * sign(absAngDeg(targetAng - getGyro)) * driveWidth / 2) / R);
        Vl = (Struct->output) * ((R - sign(arcLength) * sign(absAngDeg(targetAng - getGyro)) * driveWidth / 2) / R);
        tarAng = M_PI_2 + sign(absAngDeg(targetAng - getGyro)) * fabs(driveEnc) / R;
        driveVelo_Cor(Vl, Vr, rad2deg(tarAng), dGyro);
        // cout << Struct->error << " " << Struct->output << " " << rad2deg(tarAng) << " " << dGyro << endl;
        vexDelay(10);
    }
    drive(0);
    cout << "drift end" << endl;
}

void timerDrive(double Power, double timetarget, double gyroTarget, bool noSoftLaunch = 0)
{
    double currentRotation = getGyro;
    // double output = noSoftLaunch ? Power : 0;
    int startTime = Brain.Timer;
    // for (output = output; output <= Power; output += 2 * sign(Power))
    // {
    //     driveVelo_Cor(output > Power ? Power : output, output > Power ? Power : output, gyroTarget, getGyro);
    //     if (Brain.Timer - startTime >= timetarget)
    //     {
    //         break;
    //     }
    //     vexDelay(5);
    // }
    // cout << "TIMER " << getGyro << endl;
    while (Brain.Timer - startTime < timetarget)
    {
        // display(10, 20, "%f", time_[0]);
        drive(Power, Power, gyroTarget, getGyro);
        // cout << gyroTarget << " " << getGyro << endl;
        vexDelay(5);
    }
    // cout << "TIMEREND" << endl;
    drive(0);
}

void timerDrive(double Power, double timetarget)
{
    double currentRotation = getGyro;
    // double output = noSoftLaunch ? Power : 0;
    int startTime = Brain.Timer;
    // for (output = output; output <= Power; output += 2 * sign(Power))
    // {
    //     driveVelo_Cor(output > Power ? Power : output, output > Power ? Power : output, gyroTarget, getGyro);
    //     if (Brain.Timer - startTime >= timetarget)
    //     {
    //         break;
    //     }
    //     vexDelay(5);
    // }
    // cout << "TIMER " << getGyro << endl;
    while (Brain.Timer - startTime < timetarget)
    {
        drive(Power);
        vexDelay(5);
    }
    // cout << "TIMEREND" << endl;
    drive(0);
}

void PIDTurn(double target, double tolerance = 1, double dThresh = 0.1, int stopTime = 10000, PIDtype type = TURN)
{
    PIDtype *Struct = &type;
    Struct->insideTolerance = 0;
    // resetTimer_(0);
    int startTime = Brain.Timer;
    while ((!Struct->insideTolerance || fabs(Struct->derivative) > dThresh) && Brain.Timer - startTime < stopTime)
    {
        Struct->error = target - getGyro;
        Struct->insideTolerance = (fabs(Struct->error) <= tolerance);
        // if(Struct->insideTolerance&&fabs(Struct->lastError)>tolerance) resetTimer_(3);
        Struct->integral = (fabs(Struct->error) < Struct->iStart && Struct->error * Struct->lastError > 0) ? Struct->integral + Struct->error : 0;
        Struct->integral = cap(Struct->integral, Struct->iCap);
        Struct->derivative = IMU.gyroRate(zaxis, dps) / 100;
        Struct->lastError = Struct->error;
        Struct->output = Struct->kp * Struct->error + Struct->ki * Struct->integral + Struct->kd * Struct->derivative;
        Struct->output = cap(Struct->output, 100);
        drive(-Struct->output, Struct->output);
        vexDelay(10);
        cout << Struct->error << " " << Struct->output << " " << Struct->derivative << endl;
    }
    drive(0);
    cout << "final position " << getGyro << " " << Brain.Timer - startTime << endl;

    // display(10, 20, "final position:%f", ROTATION);
}

void LookAt(double tarX, double tarY, double bias = 0, double tolerance = 1.5, double dThresh = 0.1, int stopTime = 10000, PIDtype type = TURN)
{
    PIDtype *Struct = &type;
    Vector2d targetPoint(tarX, tarY);
    double target = 0;
    Struct->insideTolerance = 0;
    // resetTimer_(0);
    int startTime = Brain.Timer;
    while ((!Struct->insideTolerance || fabs(Struct->derivative) > dThresh) && Brain.Timer - startTime < stopTime)
    {
        Struct->error = cstrAngDeg(rad2deg((vatan2(targetPoint - Odometry.pos) - Odometry.rot) + deg2rad(bias)));
        Struct->insideTolerance = (fabs(Struct->error) < tolerance);
        // if(Struct->insideTolerance&&fabs(Struct->lastError)>tolerance) resetTimer_(3);
        Struct->integral = (fabs(Struct->error) < Struct->iStart && Struct->error * Struct->lastError > 0) ? Struct->integral + Struct->error : 0;
        Struct->integral = cap(Struct->integral, Struct->iCap);
        Struct->derivative = IMU.gyroRate(zaxis, dps) / 100;
        Struct->lastError = Struct->error;
        Struct->output = Struct->kp * Struct->error + Struct->ki * Struct->integral + Struct->kd * Struct->derivative;
        Struct->output = cap(Struct->output, 100);
        drive(-Struct->output, Struct->output);
        vexDelay(10);
        // cout << Struct->error << " " << Struct->output << " " << Struct->derivative << endl;
    }
    drive(0);
    // cout << "final position " << Odometry.rot << " " << Brain.Timer - startTime << endl;
}

bool LookAtBreak = 0;

struct LookAtParams
{
    double tarX;
    double tarY;
    double bias;
};

int LookAtUntil(double tarX, double tarY, double bias = 0, double tolerance = 1.5, double dThresh = 0.1, int stopTime = 10000, PIDtype type = TURN)
{
    PIDtype *Struct = &type;
    Vector2d targetPoint(tarX, tarY);
    double target = 0;
    Struct->insideTolerance = 0;
    // resetTimer_(0);
    int startTime = Brain.Timer;
    while (!LookAtBreak)
    {
        Struct->error = cstrAngDeg(rad2deg((vatan2(targetPoint - Odometry.pos) - Odometry.rot) + deg2rad(bias)));
        Struct->insideTolerance = (fabs(Struct->error) <= tolerance);
        // if(Struct->insideTolerance&&fabs(Struct->lastError)>tolerance) resetTimer_(3);
        Struct->integral = (fabs(Struct->error) < Struct->iStart && Struct->error * Struct->lastError > 0) ? Struct->integral + Struct->error : 0;
        Struct->integral = cap(Struct->integral, Struct->iCap);
        Struct->derivative = IMU.gyroRate(zaxis, dps) / 100;
        Struct->lastError = Struct->error;
        Struct->output = Struct->kp * Struct->error + Struct->ki * Struct->integral + Struct->kd * Struct->derivative;
        Struct->output = cap(Struct->output, 100);
        drive(-Struct->output, Struct->output);
        vexDelay(10);
        // cout << Struct->error << " " << Struct->output << " " << Struct->derivative << endl;
    }
    drive(0);
    // cout << "final position " << Odometry.rot << " " << Brain.Timer - startTime << endl;
}

void PIDDrive(double target, double tarAng = getGyro, double tolerance = 1, double dThresh = 0.12, int stopTime = 20000, PIDtype type = DRIVE)
{
    PIDtype *Struct = &type;
    Struct->optimalOutput = 30 * sign(target);
    // chassisLF.voltage(voltageUnits::mV)/100;
    Struct->insideTolerance = 0;

    LB.resetPosition();
    RB.resetPosition();

    int startTime = Brain.Timer;
    // double startGyro = getGyro;

    Struct->launching = 1;
    // cout << "drive start " << Struct->kp << endl;

    while ((!Struct->insideTolerance || fabs(Struct->derivative) > dThresh) && Brain.Timer - startTime < stopTime) //||time_[3]<400)
    {
        // Struct->launching = (fabs(driveEnc) < fabs(target / 2));
        // PIDCalc(Struct, target - driveEnc, tolerance);
        Struct->error = target - driveEnc;
        Struct->insideTolerance = (fabs(Struct->error) <= tolerance);
        // if(Struct->insideTolerance&&fabs(Struct->lastError)>tolerance) resetTimer_(3);
        Struct->integral = (fabs(Struct->error) < Struct->iStart && Struct->error * Struct->lastError > 0) ? Struct->integral + Struct->error : 0;
        Struct->derivative = -driveSpd / 100; // Struct->error - Struct->lastError;
        Struct->lastError = Struct->error;
        Struct->output = Struct->kp * Struct->error + Struct->ki * Struct->integral + Struct->kd * Struct->derivative;
        // Struct->output = Struct->output > 100 ? 100 : Struct->output;
        Struct->output = cap(Struct->output, 100);
        // if (fabs(driveEnc) > fabs(target / 2))
        //     Struct->launching = 0;
        // display(10,20,"output:%f",Struct->output);
        // display(10, 20, "%f", Struct->error);
        drive(Struct->output, Struct->output, tarAng, getGyro);
        delay(10);
        // cout << Struct->error << " " << Struct->output << "   " << Struct->derivative << endl;
    }
    drive(0);
    wipeScreen;
    // display(10, 20, "final position:%f", (encoderValue(chassisLF) + encoderValue(chassisRF)) / 2);
    // cout << "final position" << encoderValue(chassisLB) << endl;
    // cout << "Time: " << Brain.Timer - startTime << endl;
}

void PIDDrift_new(double baseSpeed, double maxDiff, double tarAng, int stopTime = 20000, double tolerance = 1)
{
    PIDtype *Struct = &TURN;
    Struct->insideTolerance = 0;
    int direction = sign(baseSpeed);
    int startTime = Brain.Timer;
    // cout << "newdrift" << endl;
    while ((!Struct->insideTolerance || Struct->derivative > 5) && Brain.Timer - startTime < stopTime)
    {
        PIDCalc(Struct, tarAng - getGyro, tolerance); // PIDCalc(Struct, absAngDeg(tarAng - getGyro), tolerance);
        Struct->output = cap(Struct->output, maxDiff);
        drive(baseSpeed - Struct->output, baseSpeed + Struct->output);
        // cout << Struct->error << " " << Struct->output << endl;
        delay(10);
    }
    // cout << "enddrift" << endl;
    drive(0);
}

void encoderDrive(double power, double target, double gyroTarget = getGyro, double stopTime = 10000)
{
    LB.resetPosition();
    RB.resetPosition();
    int startTime = Brain.Timer;
    double error = target - driveEnc, lastError = error;

    while (Brain.Timer - startTime < stopTime)
    {
        error = target - driveEnc;
        if (error * lastError < 0)
            break;
        drive(power, power, gyroTarget, getGyro);
        lastError = error;
        // cout << error << " " << lastError << endl;
        delay(10);
    }
    cout << "encoder break" << endl;
    drive(0);
}

int WingFlag = 0;

void SetWing(bool LF = 0, bool RF = 0, bool LB = 0, bool RB = 0)
{
    leftFrontWing.set(LF);
    rightFrontWing.set(RF);
    leftWing.set(LB);
    rightWing.set(RB);
}

void autowing()
{
    switch (WingFlag)
    {
    case 0:
        SetWing();
        break;
    case 1:
        SetWing(0, 0, 1, 0);
        break;
    case 2:
        SetWing();
        break;
    case 3:
        SetWing(0, 0, 1, 0);
        break;
    case 4:
        SetWing(0, 0, 0, 1);
        break;
    case 5:
        SetWing();
        break;
    case 6:
        SetWing(1, 1, 0, 0);
        break;
    case 7:
        SetWing(0, 1, 0, 0);
        break;
    case 8:
        SetWing();
        break;
    case 9:
        SetWing(1, 1, 0, 0);
        break;
    case 10:
        SetWing(0, 1, 0, 0);
        break;
    // case 11:
    //       SetWing();
    //       break;
    case 11:
        SetWing(1, 1, 0, 0);
        break;
    case 12:
        SetWing(0, 1, 0, 0);
        break;
    // case 14:
    //       //SetWing();
    //       WingFlag ++;
    //       break;
    case 13:
        SetWing(1, 1, 0, 0);
        break;
    case 14:
        SetWing();
        break;
    case 15:
        SetWing(0, 0, 0, 1);
        break;
    case 16:
        SetWing();
        break;
    case 17:
        SetWing(0, 0, 1, 0);
        break;
    case 18:
        SetWing();
        hangL.set(true);
        hangR.set(true);
        break;
    default:
        SetWing();
    }
}
#endif