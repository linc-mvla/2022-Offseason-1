#include "SwerveDrive.h"

SwerveDrive::SwerveDrive(Limelight *limelight)
{
    limelight_ = limelight;
    autoX_ = 0;
    autoY_ = 0;
}

void SwerveDrive::setYaw(double yaw)
{
    yaw_ = yaw;
}

void SwerveDrive::periodic(double yaw, Controls *controls)
{
    setYaw(yaw);
    drive(controls->getXStrafe(), controls->getYStrafe(), controls->getTurn());
}

void SwerveDrive::drive(double xSpeed, double ySpeed, double turn)
{
    calcModules(xSpeed, ySpeed, turn, false);

    // double volts = frc::SmartDashboard::GetNumber("Swerve Volts", 0.0);

    // if(abs(xSpeed) > 0.1 || abs(ySpeed) > 0.1 || abs(turn) > 0.1)
    // {
    //     topRight_->periodic(volts, trAngle_, true);
    //     topLeft_->periodic(volts, tlAngle_, true);
    //     bottomRight_->periodic(volts, brAngle_, true);
    //     bottomLeft_->periodic(volts, blAngle_, true);
    // }
    // else
    // {
    //     topRight_->periodic(0, trAngle_, true);
    //     topLeft_->periodic(0, tlAngle_, true);
    //     bottomRight_->periodic(0, brAngle_, true);
    //     bottomLeft_->periodic(0, blAngle_, true);
    // }

//0.65, 0, 0
//1, 610, 0.15534
//2, 2480, 0.63156
//3, 4600, 1.17144
//4, 6300, 1.60436
//5, 8400, 2.13915
//6, 10200, 2.59754
//m = 2.044, b = 0.669435

//turn
//1, 530, 9.66532
//2, 2400, 43.7675
//3, 4000, 72.94584
//4, 6200, 113.0661
//5, 8100, 147.7153
//6, 9800, 178.7173
//m = 0.0291946, b = 0.746574
    

    topRight_->periodic(trSpeed_, trAngle_, false);
    topLeft_->periodic(tlSpeed_, tlAngle_, false);
    bottomRight_->periodic(brSpeed_, brAngle_, false);
    bottomLeft_->periodic(blSpeed_, blAngle_, false);
}

void SwerveDrive::drivePose(double yaw, SwervePose pose)
{
    setYaw(yaw);

    /*frc::SmartDashboard::PutNumber("AX", autoX_);
    frc::SmartDashboard::PutNumber("AY", autoY_);
    frc::SmartDashboard::PutNumber("WAX", pose.getX());
    frc::SmartDashboard::PutNumber("WAY", pose.getY());  

    frc::SmartDashboard::PutNumber("WVX", pose.getXVel());
    frc::SmartDashboard::PutNumber("WVY", pose.getYVel());*/ 

    double xVel = pose.getXVel();
    double yVel = pose.getYVel();
    if(pose.getXVel() != 0 || pose.getYVel() != 0)
    {
        xVel += (pose.getX() - autoX_) * SwerveConstants::klP + pose.getXAcc() * SwerveConstants::klA;
        yVel += (pose.getY() - autoY_) * SwerveConstants::klP + pose.getYAcc() * SwerveConstants::klA;
    }
    

    double yawVel = pose.getYawVel();
    //frc::SmartDashboard::PutNumber("WYAW", yawVel);
    
    //frc::SmartDashboard::PutNumber("wy", pose.getYaw());
    if(pose.getXVel() != 0 || pose.getYVel() != 0 || pose.getYawVel() != 0)
    {
        yawVel += (pose.getYaw() - (-yaw_)) * SwerveConstants::kaP + pose.getYawAcc() * SwerveConstants::kaA;
    }
    
    
    calcModules(xVel, yVel, yawVel, true);
    
    topRight_->periodic(trSpeed_, trAngle_, true);
    topLeft_->periodic(tlSpeed_, tlAngle_, true);
    bottomRight_->periodic(brSpeed_, brAngle_, true);
    bottomLeft_->periodic(blSpeed_, blAngle_, true);
}

void SwerveDrive::calcModules(double xSpeed, double ySpeed, double turn, bool inVolts)
{
    double angle = yaw_ * M_PI / 180;

    double newX = xSpeed * cos(angle) + ySpeed * sin(angle);
    double newY = ySpeed * cos(angle) + xSpeed * -sin(angle);

    if(inVolts)
    {
        turn = (turn * M_PI / 180) * (SwerveConstants::WHEEL_DIAGONAL / 2);
    }

    double turnComponent = sqrt(turn * turn / 2);
    if(turn < 0)
    {
        turnComponent *= -1;
    }

    double A = newX - (turnComponent);
    double B = newX + (turnComponent);
    double C = newY - (turnComponent);
    double D = newY + (turnComponent);

    trSpeed_ = sqrt(B * B + C * C);
    tlSpeed_ = sqrt(B * B + D * D);
    brSpeed_ = sqrt(A * A + C * C);
    blSpeed_ = sqrt(A * A + D * D);

    if (xSpeed != 0 || ySpeed != 0 || turn != 0)
    {
        trAngle_ = -atan2(B, C) * 180 / M_PI;
        tlAngle_ = -atan2(B, D) * 180 / M_PI;
        brAngle_ = -atan2(A, C) * 180 / M_PI;
        blAngle_ = -atan2(A, D) * 180 / M_PI;
    }

    double maxSpeed;
    if(inVolts)
    {
        if(trSpeed_ != 0)
        {
            trSpeed_ = (trSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        if(tlSpeed_ != 0)
        {
            tlSpeed_ = (tlSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        if(brSpeed_ != 0)
        {
            brSpeed_ = (brSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        if(blSpeed_ != 0)
        {
            blSpeed_ = (blSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }
        
        maxSpeed = GeneralConstants::MAX_VOLTAGE;
    }
    else
    {
        maxSpeed = 1;
    }

    if (trSpeed_ > maxSpeed || tlSpeed_ > maxSpeed || brSpeed_ > maxSpeed || brSpeed_ > maxSpeed)
    {
        double max = trSpeed_;

        max = (tlSpeed_ > max) ? tlSpeed_ : max;
        max = (brSpeed_ > max) ? brSpeed_ : max;
        max = (blSpeed_ > max) ? blSpeed_ : max;

        trSpeed_ = (trSpeed_ / max);
        tlSpeed_ = (tlSpeed_ / max);
        brSpeed_ = (brSpeed_ / max);
        blSpeed_ = (blSpeed_ / max);

        if(inVolts)
        {
            trSpeed_ *= GeneralConstants::MAX_VOLTAGE;
            tlSpeed_ *= GeneralConstants::MAX_VOLTAGE;
            brSpeed_ *= GeneralConstants::MAX_VOLTAGE;
            blSpeed_ *= GeneralConstants::MAX_VOLTAGE;
        }
    }
}

void SwerveDrive::calcOdometry(double turretAngle)
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    if(!frc::DriverStation::IsEnabled())
    {
        reset();
        return;
    }

    if (!limelight_->hasTarget() && !foundGoal_ && !frc::DriverStation::IsAutonomous())
    {
        return;
    }

    double frX = -topRight_->getDriveVelocity() * sin(topRight_->getAngle() * M_PI / 180);
    double frY = topRight_->getDriveVelocity() * cos(topRight_->getAngle() * M_PI / 180);
    double flX = -topLeft_->getDriveVelocity() * sin(topLeft_->getAngle() * M_PI / 180);
    double flY = topLeft_->getDriveVelocity() * cos(topLeft_->getAngle() * M_PI / 180);
    double brX = -bottomRight_->getDriveVelocity() * sin(bottomRight_->getAngle() * M_PI / 180);
    double brY = bottomRight_->getDriveVelocity() * cos(bottomRight_->getAngle() * M_PI / 180);
    double blX = -bottomLeft_->getDriveVelocity() * sin(bottomLeft_->getAngle() * M_PI / 180);
    double blY = bottomLeft_->getDriveVelocity() * cos(bottomLeft_->getAngle() * M_PI / 180);

    double avgX = (frX + flX + brX + blX) / 4;
    double avgY = (frY + flY + brY + blY) / 4;

    double angle = yaw_ * M_PI / 180;

    double rotatedX = avgX * cos(angle) + avgY * -sin(angle);
    double rotatedY = avgX * sin(angle) + avgY * cos(angle);

    robotX_ += rotatedX * dT_;
    robotY_ += rotatedY * dT_;

    autoX_ += rotatedX * dT_;
    autoY_ += rotatedY * dT_;

    if (limelight_->hasTarget())
    {
        double distance = limelight_->calcDistance() + GeneralConstants::GOAL_RADIUS; // Origin at goal center
        robotGoalAngle_ = (180 - (turretAngle + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET));
        Helpers::normalizeAngle(robotGoalAngle_);
        double angleToGoal = yaw_ + robotGoalAngle_ + 90;
        limelightX_ = -distance * cos(angleToGoal * M_PI / 180);
        limelightY_ = -distance * sin(angleToGoal * M_PI / 180);

        double turretLimelightAngle = turretAngle - 180;
        Helpers::normalizeAngle(turretLimelightAngle);
        turretLimelightAngle = turretLimelightAngle * M_PI / 180;
        double turretLimelightX = LimelightConstants::TURRET_CENTER_RADIUS * sin(turretLimelightAngle);
        double turretLimelightY = LimelightConstants::TURRET_CENTER_RADIUS * cos(turretLimelightAngle);

        turretLimelightY -= LimelightConstants::ROBOT_TURRET_CENTER_DISTANCE;

        double robotLimelightX = turretLimelightX * cos(angle) - turretLimelightY * sin(angle);
        double robotLimelightY = turretLimelightX * sin(angle) + turretLimelightY * cos(angle);

        limelightX_ -= robotLimelightX;
        limelightY_ -= robotLimelightY;

        if (!foundGoal_ || abs(robotX_) > GeneralConstants::FIELD_WIDTH / 2 || abs(robotY_) > GeneralConstants::FIELD_LENGTH / 2 || sqrt(robotX_ * robotX_ + robotY_ * robotY_) < GeneralConstants::HUB_BASE_RADIUS)
        {
            if(abs(limelightX_) > GeneralConstants::FIELD_WIDTH / 2 || abs(limelightY_) > GeneralConstants::FIELD_LENGTH / 2 || sqrt(limelightX_ * limelightX_ + limelightY_ * limelightY_) < GeneralConstants::HUB_BASE_RADIUS)
            {
                foundGoal_ = false;
            }
            else
            {
                foundGoal_ = true;
                robotX_ = limelightX_;
                robotY_ = limelightY_;
            }
            
        }
        else
        {
            double dX = limelightX_ - robotX_;
            double dY = limelightY_ - robotY_;
            //frc::SmartDashboard::PutNumber("dx", dX);
            //frc::SmartDashboard::PutNumber("dy", dY);

            double odomRobotGoalAng;
            if (robotX_ != 0 || robotY_ != 0)
            {
                odomRobotGoalAng = -yaw_ - 90 + atan2(-robotY_, -robotX_) * 180 / M_PI;
                Helpers::normalizeAngle(odomRobotGoalAng);
            }
            else
            {
                odomRobotGoalAng = 0;
            }
            
            //double turretError = abs(180 - robotGoalAngle_ - turretAngle);
            double turretError = abs(180 - odomRobotGoalAng - turretAngle); //Test here
            Helpers::normalizeAngle(turretError);
            //frc::SmartDashboard::PutNumber("40", turretError);
            double robotGoalAngError = abs(odomRobotGoalAng - robotGoalAngle_); //Test here

            //TODO, change weight based on velocity?
            if(abs(dX) < 2 && abs(dY) < 2 && turretError < 40 && robotGoalAngError < 40 && abs(limelightX_) < GeneralConstants::FIELD_WIDTH / 2 && abs(limelightY_) < GeneralConstants::FIELD_LENGTH / 2 && sqrt(robotX_ * robotX_ + robotY_ * robotY_) > GeneralConstants::HUB_BASE_RADIUS) //Test here, higher values, originally (0.75, 0.75, 40)
            {
                robotX_ += dX * 0.05; //Test here, higher value, maybe 0.1?
                robotY_ += dY * 0.05;
                frc::SmartDashboard::PutBoolean("Swerve Using Limelight", true);
            }
            else
            {
                robotGoalAngle_ = odomRobotGoalAng;
                frc::SmartDashboard::PutBoolean("Swerve Using Limelight", false);
            }
            
        }
    }
    else
    {
        if (robotX_ != 0 || robotY_ != 0)
        {
            robotGoalAngle_ = -yaw_ - 90 + atan2(-robotY_, -robotX_) * 180 / M_PI;
            Helpers::normalizeAngle(robotGoalAngle_);


            double turretError = abs(180 - robotGoalAngle_ - turretAngle);

            /*if(abs(turretError < 20) && sqrt(robotX_ * robotX_ + robotY_ * robotY_) < 4)
            {
                foundGoal_ = false;
            }*/
        }
        else
        {
            robotGoalAngle_ = 0;
        }
    }


    frc::SmartDashboard::PutNumber("x", robotX_);
    frc::SmartDashboard::PutNumber("y", robotY_);
    //frc::SmartDashboard::PutNumber("ax", autoX_);
    //frc::SmartDashboard::PutNumber("ay", autoY_);

    // frc::SmartDashboard::PutNumber("RGA", robotGoalAngle_);
    goalXVel_ = avgX * cos(robotGoalAngle_ * M_PI / 180) + avgY * sin(robotGoalAngle_ * M_PI / 180);
    goalYVel_ = avgX * -sin(robotGoalAngle_ * M_PI / 180) + avgY * cos(robotGoalAngle_ * M_PI / 180);

}

void SwerveDrive::reset()
{
    robotX_ = 0;
    robotY_ = 0;
    autoX_ = 0;
    autoY_ = 0;
    foundGoal_ = false;
}

double SwerveDrive::getRobotGoalAng()
{
    return robotGoalAngle_;
}

double SwerveDrive::getDistance(double turretAngle)
{
    if(!foundGoal_)
    {
        return -1;
    }


    double turretLimelightAngle = turretAngle - 180;
    Helpers::normalizeAngle(turretLimelightAngle);
    turretLimelightAngle = turretLimelightAngle * M_PI / 180;
    double turretLimelightX = LimelightConstants::TURRET_CENTER_RADIUS * sin(turretLimelightAngle);
    double turretLimelightY = LimelightConstants::TURRET_CENTER_RADIUS * cos(turretLimelightAngle);

    turretLimelightY -= LimelightConstants::ROBOT_TURRET_CENTER_DISTANCE;

    double angle = yaw_ * M_PI / 180;
    double robotLimelightX = turretLimelightX * cos(angle) - turretLimelightY * sin(angle);
    double robotLimelightY = turretLimelightX * sin(angle) + turretLimelightY * cos(angle);

    double limelightToGoalX = robotX_ + robotLimelightX;
    double limelightTtoGoalY = robotY_ + robotLimelightY;
    return sqrt(limelightToGoalX * limelightToGoalX + limelightTtoGoalY * limelightTtoGoalY) - GeneralConstants::GOAL_RADIUS;
}

bool SwerveDrive::foundGoal()
{
    return foundGoal_;
}

void SwerveDrive::setFoundGoal(bool foundGoal)
{
    foundGoal_ = foundGoal;
}

double SwerveDrive::getX()
{
    return robotX_;
}

double SwerveDrive::getY()
{
    return robotY_;
}

double SwerveDrive::getGoalXVel()
{
    return goalXVel_;
}

double SwerveDrive::getGoalYVel()
{
    return goalYVel_;
}