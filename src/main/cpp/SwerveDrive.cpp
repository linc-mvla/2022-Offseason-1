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
    /*double p = frc::SmartDashboard::GetNumber("p", 0);
    frc::SmartDashboard::PutNumber("p", p);
    topRight_->setP(p);
    topLeft_->setP(p);
    bottomRight_->setP(p);
    bottomLeft_->setP(p);

    double d = frc::SmartDashboard::GetNumber("d", 0);
    frc::SmartDashboard::PutNumber("d", d);
    topRight_->setD(d);
    topLeft_->setD(d);
    bottomRight_->setD(d);
    bottomLeft_->setD(d);*/

    // calcOdometry();
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

    calcOdometry();

    double xVel = pose.getXVel();
    double yVel = pose.getYVel();
    frc::SmartDashboard::PutNumber("WX", xVel);
    frc::SmartDashboard::PutNumber("WY", yVel);
    xVel += (pose.getX() - autoX_) * SwerveConstants::klP + pose.getXAcc() * SwerveConstants::klA;
    yVel += (pose.getY() - autoY_) * SwerveConstants::klP + pose.getYAcc() * SwerveConstants::klA;

    double yawVel = pose.getYawVel();
    frc::SmartDashboard::PutNumber("WYAW", yawVel);
    
    frc::SmartDashboard::PutNumber("wy", pose.getYaw());
    yawVel += (pose.getYaw() - (-yaw_)) * SwerveConstants::kaP + pose.getYawAcc() * SwerveConstants::kaA;
    
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

    //cout << trSpeed_ << endl;

}

void SwerveDrive::calcOdometry()
{
    calcOdometry(0, true);
}

void SwerveDrive::calcOdometry(double turretAngle, bool inAuto)
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    // resetGoalOdometry(turretAngle); //TODO change into this function if it works?

    if (!limelight_->hasTarget() && !foundGoal_)
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

    // frc::SmartDashboard::PutNumber("XVEL", avgX);
    // frc::SmartDashboard::PutNumber("YVEL", avgY);

    double angle = yaw_ * M_PI / 180;

    double rotatedX = avgX * cos(angle) + avgY * -sin(angle);
    double rotatedY = avgX * sin(angle) + avgY * cos(angle);

    robotX_ += rotatedX * dT_;
    robotY_ += rotatedY * dT_;

    autoX_ += rotatedX * dT_;
    autoY_ += rotatedY * dT_;

    if (limelight_->hasTarget() && !inAuto)
    {
        //get limelight pose (so this is the position of the camera)
        frc::Pose2d limelightPose = limelight_->getPose(yaw_, turretAngle);
        limelightX_ = limelightPose.X().value();
        limelightY_ = limelightPose.Y().value();        

        //adjust to get position of actual robot
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

        //average with wheel odometry. using mechanical advantage's for now, along with basic error checking
        //error checking: only avg with limelight if values are reasonable & data is recent enough (all vals subject to change ofc)
        if (!(abs(limelightX_) > 10 || abs(limelightY_) > 10 || (frc::Timer::GetFPGATimestamp().value() - limelight_->getLastUpdated()) >= 35)) {

            //take weighted average of limelight & wheel with 4% limelight
            double averagedX = 0.04*limelightX_ + 0.96*robotX_;
            double averagedY = 0.04*limelightY_ + 0.96*robotY_;

            //what else lol?

            robotX_ = averagedX;
            robotY_ = averagedY;
        } 


        
        // if (!foundGoal_)
        // {
        //     foundGoal_ = true;
        //     robotX_ = limelightX_;
        //     robotY_ = limelightY_;
        //     // smoothX_ = limelightX_;
        //     // smoothY_ = limelightY_;
        //     // smoothWheelX_ = limelightX_;
        //     // smoothWheelY_ = limelightY_;
        // }
        // else
        // {
        //     double dX = limelightX_ - robotX_;
        //     double dY = limelightY_ - robotY_;
        //     frc::SmartDashboard::PutNumber("dx", dX);
        //     frc::SmartDashboard::PutNumber("dy", dY);
            
        //     double turretError = abs(180 - robotGoalAngle_ - turretAngle);
        //     Helpers::normalizeAngle(turretError);
        //     frc::SmartDashboard::PutNumber("40", turretError);

        //     //TODO, change weight based on velocity?
        //     if(abs(dX) < 0.75 && abs(dY) < 0.75 && turretError < 40)
        //     {
        //         robotX_ += dX * 0.05;
        //         robotY_ += dY * 0.05;
        //         frc::SmartDashboard::PutBoolean("Swerve Using Limelight", true);
        //     }
        //     else
        //     {
        //         frc::SmartDashboard::PutBoolean("Swerve Using Limelight", false);
        //     }
            
        // }

        //  double transWX = ((limelightX_ - smoothX_) + (rotatedX * dT_)) / 2;
        //  double transWY = ((limelightY_ - smoothY_) + (rotatedY * dT_)) / 2;

        // double transX = limelightX_ - smoothX_;
        // double transY = limelightY_ - smoothY_;

        // smoothWheelX_ *= 0.95;
        // smoothWheelX_ += 0.05 * transWX;

        // smoothWheelY_ *= 0.95;
        // smoothWheelY_ += 0.05 * transWY;

        // smoothX_ *= 0.95;
        // smoothX_ += 0.05 * transX;

        // smoothY_ *= 0.95;
        // smoothY_ += 0.05 * transY;
    }
    else if (!inAuto)
    {
        if (robotX_ != 0 || robotY_ != 0)
        {
            robotGoalAngle_ = -yaw_ - 90 + atan2(-robotY_, -robotX_) * 180 / M_PI;
            Helpers::normalizeAngle(robotGoalAngle_);
        }
        else
        {
            robotGoalAngle_ = 0;
        }
    }


    frc::SmartDashboard::PutNumber("x", robotX_);
    frc::SmartDashboard::PutNumber("y", robotY_);

    // frc::SmartDashboard::PutNumber("RGA", robotGoalAngle_);
    goalXVel_ = avgX * cos(robotGoalAngle_ * M_PI / 180) + avgY * sin(robotGoalAngle_ * M_PI / 180);
    goalYVel_ = avgX * -sin(robotGoalAngle_ * M_PI / 180) + avgY * cos(robotGoalAngle_ * M_PI / 180);

    /*double goalAngle = (-yaw_ - yawOffset_);
    goalAngle += 360 * 10;
    goalAngle = ((int)floor(goalAngle) % 360) + (goalAngle - floor(goalAngle));
    goalAngle -= 360 * floor(goalAngle / 360 + 0.5);

    goalAngle = goalAngle * M_PI / 180;

    goalXVel_ = avgX * cos(goalAngle) + avgY * -sin(goalAngle);
    goalYVel_ = avgX * sin(goalAngle) + avgY * cos(goalAngle);

    goalX_ += goalXVel_ * GeneralConstants::Kdt;
    goalY_ += goalYVel_ * GeneralConstants::Kdt;

    frc::SmartDashboard::PutNumber("gx", goalX_);
    frc::SmartDashboard::PutNumber("gy", goalY_);*/
}

/*void SwerveDrive::resetGoalOdometry(double turretAngle)
{
    if(!limelight_->hasTarget())
    {
        return;
    }

    foundGoal_ = true;
    goalY_ = - (limelight_->calcDistance() + 0.6096); //center of goal is origin
    goalX_ = 0;
    yawOffset_ = -yaw_ - (180 - (turretAngle + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET));
}*/

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
    /*if(foundGoal_)
    {

        double robotGoalAng = -yaw_ - yawOffset_;

        robotGoalAng += 360 * 10;
        robotGoalAng = ((int)floor(robotGoalAng) % 360) + (robotGoalAng - floor(robotGoalAng));
        robotGoalAng -= 360 * floor(robotGoalAng / 360 + 0.5);

        return robotGoalAng;
    }
    else
    {
        return 0;
    }*/

    return robotGoalAngle_;
}

double SwerveDrive::getDistance(double turretAngle)
{
    /*if (!limelight_->hasTarget())
    {
        return -1;
    }*/

    //should be equivalent? seems like all of below is done in odometry calc anyway so just getting distance via odomety should be valid?
    
    //returns distance to limelight??
    return limelight_->getDist(yaw_, turretAngle);

    // double turretLimelightAngle = turretAngle - 180;
    // Helpers::normalizeAngle(turretLimelightAngle);
    // turretLimelightAngle = turretLimelightAngle * M_PI / 180;
    // double turretLimelightX = LimelightConstants::TURRET_CENTER_RADIUS * sin(turretLimelightAngle);
    // double turretLimelightY = LimelightConstants::TURRET_CENTER_RADIUS * cos(turretLimelightAngle);

    // turretLimelightY -= LimelightConstants::ROBOT_TURRET_CENTER_DISTANCE;

    // double angle = yaw_ * M_PI / 180;
    // double robotLimelightX = turretLimelightX * cos(angle) - turretLimelightY * sin(angle);
    // double robotLimelightY = turretLimelightX * sin(angle) + turretLimelightY * cos(angle);

    // double limelightToGoalX = robotX_ + robotLimelightX;
    // double limelightTtoGoalY = robotY_ + robotLimelightY;
    // return sqrt(limelightToGoalX * limelightToGoalX + limelightTtoGoalY * limelightTtoGoalY) - GeneralConstants::GOAL_RADIUS;
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

// double SwerveDrive::getSmoothX()
// {
//     return smoothX_;
// }

// double SwerveDrive::getSmoothY()
// {
//     return smoothY_;
// }

// double SwerveDrive::getSWX()
// {
//     return smoothWheelX_;
// }

// double SwerveDrive::getSWY()
// {
//     return smoothWheelY_;
// }

/*double SwerveDrive::getGoalX()
{
    return goalX_;
}

double SwerveDrive::getGoalY()
{
    return goalY_;
}*/

double SwerveDrive::getGoalXVel() // TODO implement limelight distance if math works?
{
    /*if(limelight_->calcDistance() != -1)
    {
        return goalXVel_;
    }

    if(goalY_ == 0 && goalX_ ==0)
    {
        return 0;
    }
    double angToGoal = (atan2(-goalY_, -goalX_) * 180 / M_PI);

    double rGoalXVel = goalXVel_ * cos(angToGoal) + goalYVel_ * -sin(angToGoal);
    frc::SmartDashboard::PutNumber("RGXV", rGoalXVel);
    return rGoalXVel;*/

    frc::SmartDashboard::PutNumber("RGXV", goalXVel_);
    return goalXVel_;
}

double SwerveDrive::getGoalYVel()
{
    /*if(limelight_->calcDistance() != -1)
    {
        return goalYVel_;
    }

    if(goalY_ == 0 && goalX_ ==0)
    {
        return 0;
    }
    double angToGoal = (atan2(-goalY_, -goalX_) * 180 / M_PI);

    double rGoalYVel = goalXVel_ * sin(angToGoal) + goalYVel_ * cos(angToGoal);
    frc::SmartDashboard::PutNumber("RGYV", rGoalYVel);
    return rGoalYVel;*/

    frc::SmartDashboard::PutNumber("RGYV", goalYVel_);
    return goalYVel_;
}