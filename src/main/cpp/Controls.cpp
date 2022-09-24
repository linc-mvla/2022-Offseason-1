#include "Controls.h"

Controls::Controls() : lJoy_{InputConstants::LJOY_PORT}, rJoy_{InputConstants::RJOY_PORT}, xbox_{InputConstants::XBOX_PORT}
{
    //idk
}

void Controls::periodic()
{
    if(xbox_.GetRawButtonPressed(InputConstants::CLIMB_MODE_TOGGLE_BUTTON))
    {
        climbMode_ = !climbMode_;
        getPneumatic1Toggle();
        getPneumatic2Toggle();
        autoClimbCancelled();
        autoClimbPressed();
    }
}

double Controls::getXStrafe()
{
    /*if(manuallyOverrideTurret())
    {
        return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    }*/

    double x = lJoy_.GetRawAxis(InputConstants::LJOY_X);
    if(abs(x) < 0.05)
    {
        return 0;
    }
    x = (x > 0) ? (x - 0.05) / 0.95 : (x + 0.05) / 0.95;
    return x;
    //return (shootPressed()) ? x * 0.25 : x;

    /*if(abs(lJoy_.GetRawAxis(InputConstants::LJOY_X)) > 0.5)
    {
        return 0.1;
    }
    else
    {
        return 0;
    }*/
}

double Controls::getYStrafe()
{
    /*if(manuallyOverrideTurret())
    {
        return -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
    }*/

    double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
    if(abs(y) < 0.05)
    {
        return 0;
    }
    y = (y > 0) ? (y - 0.05) / 0.95 : (y + 0.05) / 0.95;
    return y;
    //return (shootPressed()) ? y * 0.25 : y;

    /*if(abs(lJoy_.GetRawAxis(InputConstants::LJOY_X)) > 0.5)
    {
        return 0;
    }
    else
    {
        return 0.1;
    }*/
}

double Controls::getTurn()
{
    /*if(manuallyOverrideTurret())
    {
        return 0;
    }*/

    double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
    if(abs(turn) < 0.05) //TODO get value
    {
        return 0;
    }
    turn = (turn > 0) ? ((turn - 0.05) / 0.95) * 0.3 : ((turn + 0.05) / 0.95) * 0.3;

    return turn;
    //return (shootPressed()) ? turn * 0.1 : turn;
}

bool Controls::fieldOrient()
{
    return xbox_.GetRawButton(InputConstants::FIELD_ORIENT_BUTTON);
}

double Controls::getClimbPower()
{
    /*if(abs(xbox_.GetRawAxis(InputConstants::LJOY_Y)) > 0.2)
    {
        double thing = frc::SmartDashboard::GetNumber("InCV", 0.0);
        return (xbox_.GetRawAxis(InputConstants::LJOY_Y) > 0) ? thing : -thing;
    }
    else
    {
        return 0;
    }*/
    return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y) * 0.5 * GeneralConstants::MAX_VOLTAGE;
}

bool Controls::getPneumatic1Toggle()
{
    //return xbox_.GetRawButton(InputConstants::CLIMB_PNEUMATIC2_BUTTON); //TODO change
    return xbox_.GetRawButtonPressed(InputConstants::CLIMB_PNEUMATIC1_BUTTON);
}

bool Controls::getPneumatic2Toggle()
{
    return xbox_.GetRawButtonPressed(InputConstants::CLIMB_PNEUMATIC2_BUTTON);
}

bool Controls::autoClimbPressed()
{
    return xbox_.GetRawButtonPressed(InputConstants::AUTO_CLIMB_BUTTON);
}

bool Controls::autoClimbCancelled()
{
    return xbox_.GetRawButtonPressed(InputConstants::AUTO_CLIMB_CANCEL);
}

bool Controls::intakePressed()
{
    return rJoy_.GetTrigger();
}

bool Controls::outakePressed()
{
    return rJoy_.GetRawButton(InputConstants::OUTAKE_BUTTON);
}

bool Controls::shootPressed()
{
    return lJoy_.GetTrigger()/* && resetUnload()*/;
}

bool Controls::increaseRange()
{
    return xbox_.GetRawButtonPressed(InputConstants::DISTANCE_UP_BUTTON);
}

bool Controls::decreaseRange()
{
    return xbox_.GetRawButtonPressed(InputConstants::DISTANCE_DOWN_BUTTON);
}

double Controls::getTurretManual()
{
    /*if(abs(xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X)) > 0.2)
    {
        if(xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X) > 0)
        {
            return frc::SmartDashboard::GetNumber("ITV", 0.0);
        }
        else
        {
            return -frc::SmartDashboard::GetNumber("ITV", 0.0);
        }
    }*/

    return xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X) * 0.3 * GeneralConstants::MAX_VOLTAGE;
}
//0,7 0
//0.8, 100
//1, 420
//2, 2280
//3, 4200
//4, 6100

bool Controls::resetUnload()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LTRIGGER) > 0.75;
    //cout << "resetting unload" << endl;
}

bool Controls::manuallyOverrideTurret()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_RTRIGGER) > 0.75;
}

double Controls::getHoodTicks()
{
    return abs(xbox_.GetRawAxis(InputConstants::XBOX_RJOY_Y)) * ShooterConstants::MAX_HOOD_TICKS;
}

double Controls::getTurretPos()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X) * 90;
}