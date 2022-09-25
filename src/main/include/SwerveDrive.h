#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"
#include "SwervePose.h"

#include "SwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


class SwerveDrive
{
    public:
        SwerveDrive(Limelight* limelight);
        void setYaw(double yaw);
        
        void periodic(double yaw, Controls* controls);
        void drive(double xSpeed, double ySpeed, double turn);
        void drivePose(double yaw, SwervePose pose);

        void calcModules(double xSpeed, double ySpeed, double turn, bool inVolts);

        void calcOdometry();
        void calcOdometry(double turretAngle, bool inAuto);
        //void resetGoalOdometry(double turretAngle);
        void reset();
        bool foundGoal();
        void setFoundGoal(bool foundGoal);

        double getX();
        double getY();
        // double getSmoothX();
        // double getSmoothY();
        // double getSWX();
        // double getSWY();
        //double getGoalX();
        //double getGoalY();
        double getGoalXVel();
        double getGoalYVel();
        double getRobotGoalAng();
        double getDistance(double turretAngle);
    private:
        SwerveModule* topRight_ = new SwerveModule(SwerveConstants::TR_TURN_ID, SwerveConstants::TR_DRIVE_ID, SwerveConstants::TR_CANCODER_ID, SwerveConstants::TR_CANCODER_OFFSET);
        SwerveModule* topLeft_ = new SwerveModule(SwerveConstants::TL_TURN_ID, SwerveConstants::TL_DRIVE_ID, SwerveConstants::TL_CANCODER_ID, SwerveConstants::TL_CANCODER_OFFSET);
        SwerveModule* bottomRight_ = new SwerveModule(SwerveConstants::BR_TURN_ID, SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_CANCODER_ID, SwerveConstants::BR_CANCODER_OFFSET);
        SwerveModule* bottomLeft_ = new SwerveModule(SwerveConstants::BL_TURN_ID, SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_CANCODER_ID, SwerveConstants::BL_CANCODER_OFFSET);

        double limelightX_, limelightY_, robotX_, robotY_, autoX_, autoY_, yaw_, goalXVel_, goalYVel_, robotGoalAngle_;
        //double smoothX_, smoothY_, smoothWheelX_, smoothWheelY_;
        bool foundGoal_ = false;

        double prevTime_, dT_;

        Limelight* limelight_;

        frc::Timer timer_;

        double trSpeed_, brSpeed_, tlSpeed_, blSpeed_, trAngle_, brAngle_, tlAngle_, blAngle_;

};