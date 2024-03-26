#ifndef _DEVICES_H_
#define _DEVICES_H_

#include "vex.h"
#include "mathFunctions.h"

using namespace vex;

using namespace std;

brain Brain = brain();

controller Controller = controller();

motor CataL = motor(PORT1, 1); // 13
motor CataR = motor(PORT10);   // 13·

motor_group Cata = motor_group(CataR, CataL);

motor Itk = motor(PORT1, ratio18_1, 1);

motor LF = motor(PORT2, ratio6_1, 1);
motor LM = motor(PORT3, ratio6_1); // 4
motor LB = motor(PORT4, ratio6_1, 1);
motor RF = motor(PORT9, ratio6_1);
motor RM = motor(PORT8, ratio6_1, 1);
motor RB = motor(PORT7, ratio6_1);
triport Triport = triport(PORT1);

rotation TX = rotation(PORT11);
rotation TY = rotation(PORT11);

digital_out leftWing = digital_out(Brain.ThreeWirePort.C);
digital_out rightWing = digital_out(Brain.ThreeWirePort.E);
digital_out PTO = digital_out(Brain.ThreeWirePort.D);

digital_out leftFrontWing = digital_out(Brain.ThreeWirePort.B);
digital_out rightFrontWing = digital_out(Brain.ThreeWirePort.G);
digital_out hangL = digital_out(Brain.ThreeWirePort.A);
digital_out hangR = digital_out(Brain.ThreeWirePort.F);

// inertial IMU = inertial(PORT12, vex::turnType::left);
class Gyr : public inertial
{
public:
  Gyr(int32_t index, turnType dir = turnType::right) : inertial(index, dir) {}
  void setRate(int mscount)
  {
    datarate(mscount);
  }
};

Gyr IMU = Gyr(PORT17, vex::turnType::left);
#define robotX ((TX.position(rotationUnits::deg) / 360) * 21.96)
#define robotY ((TY.position(rotationUnits::deg) / 360) * 21.96)

// #define getGyro deg2rad(IMU.rotation(rotationUnits::raw) * 1.016949152542372)
// #define getGyroDeg IMU.rotation(rotationUnits::raw) * 1.016949152542372

#define getGyro (IMU.rotation(deg) * 1.0053) // 1.0039040713887339654210819854992)

#define getGyroRad deg2rad(getGyro)

#define wipeScreen Brain.Screen.clearScreen()
#define display Brain.Screen.printAt
#define controllerSetCursor Controller.Screen.setCursor
#define controllerWipeScreen Controller.Screen.clearScreen()
#define controllerPrint Controller.Screen.print

bool skillsCata = 0, modeUnder = 0;
bool first = 1;
bool skillsRun = 0;
#endif