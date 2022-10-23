#pragma once

#include <frc/Joystick.h>
#include "Constants.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

class Controls
{
    public:
        Controls();
        void periodic();

        double getXStrafe();
        double getYStrafe();
        double getTurn();

        bool fieldOrient();

        double getClimbPower();
        bool getPneumatic1Toggle();
        bool getPneumatic2Toggle();
        bool autoClimbPressed();
        bool autoClimbCancelled(); //Check if autoclimb cancle button is pressed

        bool intakePressed();
        bool outakePressed();

        bool shootPressed();
        double getTurretManual();

        bool increaseRange();
        bool decreaseRange();

        bool getClimbMode(){ return climbMode_; }
        void setClimbMode(bool climbMode){ climbMode_ = climbMode; }//Set climb mode

        bool resetUnload();
        bool manuallyOverrideTurret();

        double getHoodTicks();
        double getTurretPos();

    private:
        bool climbMode_; //If robot is going to climb

        frc::Joystick lJoy_;
        frc::Joystick rJoy_;
        frc::Joystick xbox_;
};