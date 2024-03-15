#ifndef _TASKS_H_
#define _TASKS_H_

#include "vex.h"
#include "devices.h"
#include "mathFunctions.h"
#include "functions.h"

float lockTarget = 20;

/*int cataCtrl()
{
    float error = lockTarget - Cata.position(deg);
    bool lastModeUnder = 0;
    moveCata(-20);
    delay(1000);
    moveCata(0);
    delay(1000);
    Cata.resetPosition();
    while (1)
    {
        error = lockTarget - Cata.position(deg);
        // if (BB.PRESSED)
        //   lockTarget += 180;
        // if (BB.RELEASED)
        //   lockTarget += 20;
        // if (BB.PRESSED)
        // {
        //   lockTarget += 160;
        //   primed = 1;
        // }
        // if (BB.RELEASED && primed)
        // {
        //   lockTarget += 40;
        //   delay(2000);
        //   lockTarget += 160;
        //   primed = 0;
        // }

        if (BB.PRESSED)
            lockTarget += 180;

        if (BA.PRESSED)
        {
            modeUnder = !modeUnder;
            // lockTarget += 60;
        }
        if (modeUnder && !lastModeUnder)
            lockTarget += 140;
        else if (!modeUnder && lastModeUnder)
            lockTarget += 40;
        // if (BA.RELEASED)
        //     lockTarget += 120;

        if (BL.PRESSED)
            skillsCata = !skillsCata;

        if (!skillsCata)
        {
            if (fabs(error) < 5)
            {
                if (error >= 0)
                    moveCata(error * 0.3 + 0.6 * (Cata.position(deg) + 180 - lockTarget));
                else
                    moveCata(0.25 * (Cata.position(deg) + 180 - lockTarget));
            }
            else if (error >= 0)
                moveCata(100);
            else
                moveCata(0);
        }
        else
            moveCata(82.5);

        lastModeUnder = modeUnder;
        delay(10);
    }
    return 0;
}*/

// int cataCtrl()
// {

//     return 0;
// }

bool getCorner = 0;

int closeWingHook()
{
    while (1){
         if (getCorner){
            while (getGyro < 140){
                delay(10);
            }
            getCorner = 0;
            closeLeft;
         }
    delay(10);
    }

}

#endif