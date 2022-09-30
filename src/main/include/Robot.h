// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include "AHRS.h"
#include "Constants.h"
#include "Controls.h"
#include "SwerveDrive.h"
#include "Intake.h"
#include "Shooter.h"
#include "Limelight.h"
#include "Climb.h"
#include "Channel.h"
#include "Logger.h"
#include "AutoPaths.h"

class Robot : public frc::TimedRobot
{
public:
    Robot();
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;

private:
    frc::SendableChooser<AutoPaths::Path> autoChooser_; //Sets choices for auto
    frc::DriverStation::Alliance alliance_;

    AHRS *navx_;
    Limelight* limelight_ = new Limelight();

    Controls* controls_ = new Controls();
    SwerveDrive* swerveDrive_ = new SwerveDrive(limelight_);
    Channel* channel_ = new Channel();
    Shooter* shooter_ = new Shooter(limelight_, swerveDrive_, channel_);
    Intake intake_;
    Climb climb_;
    AutoPaths autoPaths_;

    //TODO test, also make not a pointer
    //Logger* odometryLogger_ = new Logger(OutputConstants::odometryFile);
    //Logger* flywheelLogger_ = new Logger(OutputConstants::flywheelFile);
    Logger* hoodLogger_ = new Logger(OutputConstants::hoodFile);
    //Logger* turretLogger_ = new Logger(OutputConstants::turretFile);

    frc::Timer climbTimer_;

    double yawOffset_;

};
