#include "SwerveDrive.h"

SwerveDrive::SwerveDrive(Limelight* limelight)
{
    limelight_ = limelight;
}

void SwerveDrive::setYaw(double yaw)
{
    yaw_ = yaw;
}

void SwerveDrive::periodic(double yaw, Controls* controls)
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

    //calcOdometry();
    calcModules(xSpeed, ySpeed, turn);

    topRight_->periodic(trSpeed_, trAngle_);
    topLeft_->periodic(tlSpeed_, tlAngle_);
    bottomRight_->periodic(brSpeed_, brAngle_);
    bottomLeft_->periodic(blSpeed_, blAngle_);

}

void SwerveDrive::drivePose(double yaw, SwervePose pose)
{
    setYaw(yaw);


}

void SwerveDrive::calcModules(double xSpeed, double ySpeed, double turn)
{
    double angle = yaw_ * M_PI / 180;

    double newX = xSpeed * cos(angle) + ySpeed * sin(angle);
    double newY = ySpeed * cos(angle) + xSpeed * -sin(angle);

    double A = newX - (turn);
    double B = newX + (turn);
    double C = newY - (turn);
    double D = newY + (turn);

    trSpeed_ = sqrt(B*B + C*C);
    tlSpeed_ = sqrt(B*B + D*D);
    brSpeed_ = sqrt(A*A + C*C);
    blSpeed_ = sqrt(A*A + D*D);

    if(xSpeed != 0 || ySpeed != 0 || turn != 0)
    {
        trAngle_ = -atan2(B, C) * 180 / M_PI;
        tlAngle_ = -atan2(B, D) * 180 / M_PI;
        brAngle_ = -atan2(A, C) * 180 / M_PI;
        blAngle_ = -atan2(A, D) * 180 / M_PI;
    }

    if(trSpeed_ > 1 || tlSpeed_ > 1 || brSpeed_ > 1 || brSpeed_ > 1)
    {
        double max = trSpeed_;

        max = (tlSpeed_ > max) ? tlSpeed_ : max;
        max = (brSpeed_ > max) ? brSpeed_ : max;
        max = (blSpeed_ > max) ? blSpeed_ : max;

        trSpeed_ = (trSpeed_ / max);
        tlSpeed_ = (tlSpeed_ / max);
        brSpeed_ = (brSpeed_ / max);
        blSpeed_ = (blSpeed_ / max);
    }
}

void SwerveDrive::calcOdometry(double turretAngle)
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;
    
    //resetGoalOdometry(turretAngle); //TODO change into this function if it works?

    if(!limelight_->hasTarget() && !foundGoal_)
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

    //frc::SmartDashboard::PutNumber("XVEL", avgX);
    //frc::SmartDashboard::PutNumber("YVEL", avgY);

    double angle = yaw_ * M_PI / 180;
    
    double rotatedX = avgX * cos(angle) + avgY * -sin(angle);
    double rotatedY = avgX * sin(angle) + avgY * cos(angle);

    if(limelight_->hasTarget())
    {   
        double distance = limelight_->calcDistance() + 0.6096; //Origin at goal center
        robotGoalAngle_ = (180 - (turretAngle + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET));
        double angleToGoal = yaw_ + robotGoalAngle_ + 90;
        x_ = -distance * cos(angleToGoal * M_PI / 180);
        y_ = -distance * sin(angleToGoal * M_PI / 180);

        double turretLimelightAngle = turretAngle - 180;
        Helpers::normalizeAngle(turretLimelightAngle);
        turretLimelightAngle = turretLimelightAngle * M_PI / 180;
        double turretLimelightX = LimelightConstants::TURRET_CENTER_RADIUS * sin(turretLimelightAngle);
        double turretLimelightY = LimelightConstants::TURRET_CENTER_RADIUS * cos(turretLimelightAngle);

        turretLimelightY -= LimelightConstants::ROBOT_TURRET_CENTER_DISTANCE;

        double robotLimelightX = turretLimelightX * cos(angle) - turretLimelightY * sin(angle);
        double robotLimelightY = turretLimelightX * sin(angle) + turretLimelightY * cos(angle);

        x_ -= robotLimelightX;
        y_ -= robotLimelightY;

        if(!foundGoal_)
        {
            foundGoal_ = true;
            smoothX_ = x_;
            smoothY_ = y_;
            smoothWheelX_ = x_;
            smoothWheelY_ = y_;
        }

        //TODO average with wheel velocity?
        double transWX = ((x_ - smoothX_) + (rotatedX * dT_)) / 2;
        double transWY = ((y_ - smoothY_) + (rotatedY * dT_)) / 2;

        double transX = x_ - smoothX_;
        double transY = y_ - smoothY_;

        smoothWheelX_ *= 0.95;
        smoothWheelX_ += 0.05 * transWX;

        smoothWheelY_ *= 0.95;
        smoothWheelY_ += 0.05 * transWY;

        smoothX_ *= 0.95;
        smoothX_ += 0.05 * transX;

        smoothY_ *= 0.95;
        smoothY_ += 0.05 * transY;

    }
    else
    {
        x_ += rotatedX * dT_;
        y_ += rotatedY * dT_;

        if(x_ != 0 || y_ != 0)
        {
            robotGoalAngle_ = -yaw_ - 90 + atan2(-y_, -x_) * 180 / M_PI;
        }
        else
        {
            robotGoalAngle_ = 0;
        }
    }

    Helpers::normalizeAngle(robotGoalAngle_);

    frc::SmartDashboard::PutNumber("x", x_);
    frc::SmartDashboard::PutNumber("y", y_);

    //frc::SmartDashboard::PutNumber("RGA", robotGoalAngle_);
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
    x_ = 0;
    y_ = 0;
    //goalX_ = 0;
    //goalY_ = 0;
    //yawOffset_ = 0;
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
    return x_;
}

double SwerveDrive::getY()
{
    return y_;
}

double SwerveDrive::getSmoothX()
{
    return smoothX_;
}

double SwerveDrive::getSmoothY()
{
    return smoothY_;
}

double SwerveDrive::getSWX()
{
    return smoothWheelX_;
}

double SwerveDrive::getSWY()
{
    return smoothWheelY_;
}

/*double SwerveDrive::getGoalX()
{
    return goalX_;
}

double SwerveDrive::getGoalY()
{
    return goalY_;
}*/

double SwerveDrive::getGoalXVel() //TODO implement limelight distance if math works?
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