#include <Limelight.h>
#include <iostream>


Limelight::Limelight(){
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

//moved distance calculator to shooter calc

//Return the X offset to the target
double
Limelight::getXOff(){
    return table->GetNumber("tx", 10000.0);
}


//Return the y offset to the target
double
Limelight::getYOff(){
    return table->GetNumber("ty", 10000.0);
}


void Limelight::adjustAngles(double& ax, double& ay)
{
    double flippedY = (90 - ay) * M_PI / 180;
    double flippedX = (90 - ax) * M_PI / 180;

    //Spherical to cartesian
    double x = sin(flippedY) * cos(flippedX);
    double y = sin(flippedY) * sin(flippedX);
    double z = cos(flippedY);
    
    //TODO: confirm?
    double rotAng = LimelightConstants::CAM_ANGLE * M_PI / 180;

    //rotate
    double rotatedY = y * cos(rotAng) - z * sin(rotAng);
    double rotatedZ = y * sin(rotAng) + z * cos(rotAng);

    //get new angles
    double horizontal = sqrt(x * x + rotatedY * rotatedY);
    double nay, nax;
    if(rotatedZ != 0 || horizontal != 0) {
        nay = atan2(rotatedZ, horizontal) * 180 / M_PI;
    }
    else nay = 0;

    if(rotatedY != 0 || x != 0) {
        nax = (atan2(x, rotatedY) * 180 / M_PI);
    }
    else nax = 0;

    ax = nax;
    ay = nay;
}

double Limelight::getAdjustedX()
{
    double x = getXOff();
    double y = getYOff();
    adjustAngles(x, y);

    frc::SmartDashboard::PutNumber("LAX", x);
    frc::SmartDashboard::PutNumber("LAY", y);

    return x;
}

//coordinates: gonna assume angle is zero when robot facing directly away
frc::Pose2d Limelight::getPose(double navx, double turretAngle) {
    double distance = calcDistance() + 0.686;
    double robotGoalAngle_ = -(turretAngle + getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET); 
    double angleToGoal = navx + robotGoalAngle_;
    double y = -distance * cos(angleToGoal * M_PI / 180);
    double x = distance * sin(angleToGoal * M_PI / 180);

   // frc::SmartDashboard::PutNumber("distance", distance);
    // frc::SmartDashboard::PutNumber("robotGoalAngle", robotGoalAngle_);
    // frc::SmartDashboard::PutNumber("angleToGoal", angleToGoal);
    // frc::SmartDashboard::PutNumber("Pose x", x);
    // frc::SmartDashboard::PutNumber("Pose y", y);


    // std::vector<double> v = network_table->GetNumberArray("tcornxy", wpi::span<double, std::size_t{0}>{});
    // for (double d : v) std::cout << d << "\n";
    // std::cout << "\n";


    return frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{units::degree_t{navx}}};
}

double Limelight::calcDistance() {
    double x = getXOff();
    double y = getYOff();
    adjustAngles(x, y);
    return (GeneralConstants::GOAL_HEIGHT - LimelightConstants::HEIGHT_OFFSET) / tan(y * M_PI / 180);
}

bool Limelight::hasTarget() {
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



void Limelight::lightOn(bool light)
{
    if(light){
        table->PutNumber("ledMode", 3);
    }
    else{
        table->PutNumber("ledMode", 1);
    }
}