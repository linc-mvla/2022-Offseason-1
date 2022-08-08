#include "Limelight.h"


Limelight::Limelight()
{
    table = nt::NetworkTableInstance::GetDefault().GetTable(tableName);
    //table->PutNumber("pipeline", PIPELINE);
}

double Limelight::getXOff()
{
    //frc::SmartDashboard::PutNumber("LX", table->GetNumber("tx", 10000.0));
    return table->GetNumber("tx", 10000.0);
}

double Limelight::getYOff()
{
    //frc::SmartDashboard::PutNumber("LY", table->GetNumber("ty", 10000.0));
    return table->GetNumber("ty", 10000.0);
}

double Limelight::getAdjustedX()
{
    double x = getXOff();
    double y = getYOff();
    adjustAngles(x, y);

    //frc::SmartDashboard::PutNumber("LAX", x);
    //frc::SmartDashboard::PutNumber("LAY", y);

    return x;
}

bool Limelight::hasTarget()
{
    double targets = table->GetNumber("tv", -1);

    //std::vector<double> thing = table->GetEntry("llpython").GetDoubleArray(std::vector<double>());
    //cout << thing[0] << endl;

    if(targets == -1 || targets == 0)
    {
        return false;
    } 
    else 
    {
        return true;
    }
}

void Limelight::adjustAngles(double& ax, double& ay)
{
    double flippedY = (90 - ay) * M_PI / 180;
    double flippedX = (90 - ax) * M_PI / 180;

    //Spherical to cartesian
    double x = sin(flippedY) * cos(flippedX);
    double y = sin(flippedY) * sin(flippedX);
    double z = cos(flippedY);
    
    double rotAng = LimelightConstants::ANGLE_OFFSET * M_PI / 180;

    //rotate
    double rotatedY = y * cos(rotAng) - z * sin(rotAng);
    double rotatedZ = y * sin(rotAng) + z * cos(rotAng);

    //get new angles
    double horizontal = sqrt(x * x + rotatedY * rotatedY);
    double nay, nax;
    if(rotatedZ != 0 || horizontal != 0)
    {
        nay = atan2(rotatedZ, horizontal) * 180 / M_PI;
    }
    else
    {
        nay = 0;
    }

    if(rotatedY != 0 || x != 0)
    {
        nax = (atan2(x, rotatedY) * 180 / M_PI);
    }
    else
    {
        nax = 0;
    }

    ax = nax;
    ay = nay;
}

double Limelight::calcDistance()
{
    if(!hasTarget())
    {
        return -1;
    }

    double x = getXOff();
    double y = getYOff();
    adjustAngles(x, y);
    return (GeneralConstants::GOAL_HEIGHT - LimelightConstants::HEIGHT_OFFSET) / tan(y * M_PI / 180);
}

void Limelight::lightOn(bool light)
{
    if(light){
        table->PutNumber("ledMode", 3);
    }
    else{
        table->PutNumber("ledMode", 1);
    }
}