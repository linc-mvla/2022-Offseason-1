#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"
#include <AHRS.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include "SwerveModule.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>


class SwerveDrive
{
    public:
        SwerveDrive(AHRS * nx, Limelight* limelight); //todo: add logger
        void setYaw(double yaw);
        
        void Periodic(units::meters_per_second_t joy_x, units::meters_per_second_t joy_y, 
        units::radians_per_second_t joy_theta, units::degree_t navx_yaw);

        void initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose);
        void updateOdometry(frc::Rotation2d robotAngle, frc::Pose2d robotPose) { odometry_->ResetPosition(robotPose, robotAngle); }

        wpi::array<frc::SwerveModuleState, 4> getRealModuleStates(); //real as upposed to goal

        void calcModules(double xSpeed, double ySpeed, double turn);

        void updateLimelightOdom(double turretAngle, bool inAuto);
        //void resetGoalOdometry(double turretAngle);
        void reset();
        bool foundGoal();
        void setFoundGoal(bool foundGoal);

        double getDistance(double turretAngle);

        double getX();
        double getY();
        double getSmoothX();
        double getSmoothY();
        double getSWX();
        double getSWY();
        //double getGoalX();
        //double getGoalY();
        double getGoalXVel();
        double getGoalYVel();
        double getRobotGoalAng();
    private:
        AHRS * m_navx;
        Limelight* limelight_;
        frc::SwerveDriveOdometry<4> * odometry_; //will need to be initialized later with selected robot start pose

        frc::ChassisSpeeds speeds_;

        frc::ChassisSpeeds getRobotSpeeds();


        frc::SwerveDriveKinematics<4> m_kinematics{
            frc::Translation2d{0.3683_m, 0.3683_m}, frc::Translation2d{0.3683_m, -0.3683_m},
            frc::Translation2d{-0.3683_m, 0.3683_m}, frc::Translation2d{-0.3683_m, -0.3683_m}
        };


        SwerveModule flModule_{SwerveConstants::FLanglePort, SwerveConstants::FLspeedPort, SwerveConstants::FLencoder, false, SwerveConstants::FLOFF};
        SwerveModule frModule_{SwerveConstants::FRanglePort, SwerveConstants::FRspeedPort, SwerveConstants::FRencoder, true, SwerveConstants::FROFF};
        SwerveModule blModule_{SwerveConstants::BLanglePort, SwerveConstants::BLspeedPort, SwerveConstants::BLencoder, true, SwerveConstants::BLOFF};
        SwerveModule brModule_{SwerveConstants::BRanglePort, SwerveConstants::BRspeedPort, SwerveConstants::BRencoder, false, SwerveConstants::BROFF};

        frc2::PIDController angPID_{SwerveConstants::P , SwerveConstants::I, SwerveConstants::D};
        frc2::PIDController speedPID_{SwerveConstants::sP , SwerveConstants::sI, SwerveConstants::sD};

        
        double limelightX_, limelightY_, yaw_, /*goalX_, goalY_, yawOffset_,*/ goalXVel_, goalYVel_, robotGoalAngle_;
        double smoothX_, smoothY_, smoothWheelX_, smoothWheelY_;
        bool foundGoal_ = false;

        double prevTime_, dT_;

        frc::Timer timer_;

        double trSpeed_, brSpeed_, tlSpeed_, blSpeed_, trAngle_, brAngle_, tlAngle_, blAngle_;

};