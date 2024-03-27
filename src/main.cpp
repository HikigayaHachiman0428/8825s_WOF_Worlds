#include "devices.h"
#include "vex.h"
#include "functions.h"
#include "autonomous.h"
#include "tasks.h"
#include "bezier_curve.h"
#include "pure_pursuit.h"
#include "odometry.h"

using namespace vex;

using namespace std;
competition Competition;

void pre_auton(void)
{
}

int autoStartTime = 0;

void autonomous(void)
{
  outer_alliance();
  // innerElimination();
  // innerFAST();
  // _15s_out_quali_();
  // _15s_Out_arx_7_();
  // PIDDrive(-70, 0);
  // PIDDrift_new(30, 70, -90);
  // _15s_in_6balls_safe();
  // _15s_in_quali_();
  // testBezier();
  // skills_with_odom();
  // skills_with_odom_stiff_goal();
  // genius_route();
  // PIDDrive(60);
  // PIDTurn(90);
  // encoderDrive(20, 10000, 0, 500);
  // first = 0;
  // skills();
  // finishedAuto = 1;
  // autoStartTime = Brain.timer(msec);

  // cout << "time used: " << Brain.timer(msec) - autoStartTime << endl;
  // // PIDDrift(-30, 100, 1600);
  // // Kcor = 0;
  // // PIDDrift(50, 90);
  // // PIDDrive(50);
  // // driveVelo_Cor(10, 10, 10, -10);
  // // PIDDrift(1, 1, 80, 70, 2);
  // // drive(-10, 10);
  // // _15s_in();
  // // _15s_out_sawp();
  // // PIDTurn(90);
  // // cout<<(((90.0 - ((double)IMU.rotation(deg))) / 180.0f) * 3.14159265358979323846)<<" "<<deg2rad(90.0 - getGyro)<<" "<<getGyro<<endl;
}

// user functions and definitions

int printInfo()
{
  while (1)
  {
    // cout<<Odometry.pos.transpose()<<endl;
    // cout<<Odometry.rot<<endl;
    // cout<<robotX<<" "<<robotY<<endl;
    // cout<<Odometry.trackingPos.transpose()<<endl;
    // wipeScreen;
    Controller.Screen.clearScreen();
    // Controller.Screen.print(Cata.temperature(temperatureUnits::celsius));
    Controller.Screen.setCursor(1, 0);
    controllerPrint(getGyro);
    Controller.Screen.setCursor(2, 0);
    // if (skillsRun)
    if (debug)
      controllerPrint("debug           ");
    else
      controllerPrint("normal run with matchload     ");
    // else
    //   controllerPrint("normal");
    // controllerPrint();
    // controllerPrint(Cata.position(deg));
    // wipeScreen;
    display(10, 20, "                                                                  ");
    display(10, 60, "               Gyro: %.2f                                         ", getGyro);
    display(10, 40, "drive: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f",
            LF.temperature(celsius), LF.temperature(celsius), LF.temperature(celsius),
            LF.temperature(celsius), LF.temperature(celsius), LF.temperature(celsius));
    display(10, 60, "%.1f", Cata.temperature(celsius));
    if (BA.pressing())
    {
      display(10, 100, "x: %.1f, y: %.1f, rot: %.1f", Odometry.pos[0], Odometry.pos[1], Odometry.rot);
      display(10, 80, "xEnc: %.1f, yEnc: %.1f, Gyro: %.1f ", robotX, robotY, getGyro);
      // cout<<robotX<<" "<<robotY<<endl;
      cout << Odometry.pos.transpose() << endl;
    }
    switch (auton)
    {
    case 0:
      display(10, 20, "IN_QUALI");
      break;
    case 1:
      display(10, 20, "IN_ELIM");
      break;
    case 2:
      display(10, 20, "OUT_QUALI");
      break;
    case 3:
      display(10, 20, "OUT_ELIM");
      break;
    case 4:
      display(10, 20, "debug");
      break;
    default:
      display(10, 20, "ERROR");
      break;
    }
    vexDelay(10);
  }
  return 0;
}

void usercontrol(void)
{
  PurePursuitFlag = 0;
  leftWing.set(0);
  rightWing.set(0);
  leftFrontWing.set(0);
  rightFrontWing.set(0);
  // User control code here, inside the loop

  bool primed = 0;

  bool wingsOpen = 0;

  bool awpShoot = 0;

  bool hanging = 1;

  // bool hang = 0;

  bool cataPractice = 0;
  bool spin = 1;

  double cataTarget = 0;

  bool lastJoyRight = false;
  bool lastJoyLeft = false;
  bool JoyRight = false;
  bool JoyLeft = false;
  bool JoyTop = false;
  bool JoyBottom = false;
  bool lastJoyTop = false;
  bool lastJoyBottom = false;
  bool autoWingFlag = false;

  PurePursuitFlag = 0;

  // task skills_(genius_route);
  //  while (1)
  // {
  //   if (BX.RELEASED)
  //     break;
  //   delay(10);
  // }
  // skills_.stop();
  PurePursuitFlag = 0;
  // int cataMode = 0, lastCataMode = 0; // 0 driver 1 skills 2 matchload
  while (1)
  {
    // if (Cata.position(deg) < cataTarget)
    //   moveCata(100 * hanging);
    // else
    //   moveCata(0 * hanging);
    // if (L1.PRESSED)
    // {
    //   cataTarget += 360;
    //   // cataPractice = !cataPractice;
    //   // moveCata(92 * cataPractice);
    //   // if(cataPractice) Cata.spin(fwd, 90, rpm);
    //   // else moveCata(0);
    // }
    moveCata(100 * BA.pressing());

    if (R1.pressing() && !R2.pressing())
      moveItk(100);
    else if (!R1.pressing() && R2.pressing())
      moveItk(-100);
    else
      moveItk(20);

    if (BU.PRESSED)
    {
      hangL.set(!hangL.value());
      hangR.set(!hangR.value());
    }
    // auton++;
    // if (BL.PRESSED && auton > 0)
    //   auton--;

    // driveLock = BX.pressing();
    // if (BR.pressing())
    //   cout << Odometry.pos.transpose() << endl;
    // if (L1.PRESSED)
    //   Odometry.setPose(135, -120);
    JoyLeft = Controller.Axis4.position() < -80;
    JoyRight = Controller.Axis4.position() > 80;
    JoyTop = Controller.Axis3.position() > 80;
    JoyBottom = Controller.Axis3.position() < -80;

    if (JoyBottom && !lastJoyBottom)
    {
      autoWingFlag = !autoWingFlag;
    }

    if (autoWingFlag == false)
    {
      if (Controller.ButtonL1.PRESSED)
      {
        WingFlag++;
        autowing();
      }
      else if (Controller.ButtonL2.PRESSED)
      {
        WingFlag--;
        autowing();
      }
    }
    else
    {
      if (Controller.ButtonL1.PRESSED)
      {
        leftWing.set(!leftWing.value());
      }

      if (Controller.ButtonL2.PRESSED)
      {
        rightWing.set(!rightWing.value());
      }
      if (JoyTop)
      {
        leftFrontWing.set(1);
        rightFrontWing.set(1);
      }
      else
      {
        if (JoyLeft)
        {
          leftFrontWing.set(true);
        }
        else
          leftFrontWing.set(0);

        if (JoyRight)
        {
          rightFrontWing.set(true);
        }
        else
          rightFrontWing.set(0);
      }
    }

    lastJoyLeft = JoyLeft;
    lastJoyRight = JoyRight;
    lastJoyTop = JoyTop;
    lastJoyBottom = JoyBottom;
    // cout<<fabs(pow(JLR * 0.01, 1.2)) * 100 * sign(JLR)<<endl;

    // driveLeft(JFB + fabs(pow(fabs(JLR) * 0.01, 1.1)) * 100 * sign(JLR));
    // driveRight(JFB - fabs(pow(fabs(JLR) * 0.01, 1.1)) * 100 * sign(JLR));
    driveLeft(JFB + JLR);
    driveRight(JFB - JLR);
    // cout << lockTarget << " " << Cata.position(deg) << endl;
    // if (L1.PRESSED)
    //   leftWing.set(!leftWing.value());
    //  // leftWing.set(!leftWing.value());
    // if (L2.PRESSED)
    //   rightWing.set(!rightWing.value());
    //   if (BX.PRESSED)
    //     break;
    //   delay(10);
    // }
    // skills_.stop();
    // PIDTurn(90);
    // Shoot.set(!Shoot.value());
    delay(10);
  }
}

int main()
{
  // Matrix<double, 4, 2> testWayPoints[1];
  // double speedtest[1];
  // speedtest[0] = 1;
  // testWayPoints[0] = Matrix<double, 4, 2>{
  //     {0, 0},
  //     {10, 80},
  //     {20, 0},
  //     {30, -80}};
  // // Vector2f testVec;
  // // testVec << 1.1, 2.2;
  // // Matrix<double, 4, 2> display_ = (testWayPoints)[0];
  // // cout << display_ << endl;
  // CubicBezier testCurve = CubicBezier(testWayPoints, speedtest, 1);
  // testCurve.BezierPrint();
  // cout<<scanIntersection(Vector2d(20, 5), &testCurve, 0)<<endl;
  // cout<<testCurve.BezierPoint(scanIntersection(Vector2d(20, 5), &testCurve, 0), 0).transpose()<<endl;
  // cout<<testVec(1)<<endl;
  init();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // task cataCtrl_(cataCtrl);
  task printInfo_(printInfo);
  // task lockDrive_(lockDrive);

  task odometry_(trackingWrapper);

  task autoDrive_(PurePursuitControl);

  pre_auton();
  while (true)
  {
    // cout << getGyro << endl;
    // cout << IMU.roll(deg) << endl;
    wait(100, msec);
  }
}
