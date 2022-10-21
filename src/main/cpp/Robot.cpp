// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() : autoPaths_(channel_)
{

    AddPeriodic(
        [&]
        {
            double yaw = navx_->GetYaw() - yawOffset_;
            Helpers::normalizeAngle(yaw);

            if(frc::DriverStation::IsAutonomous() && frc::DriverStation::IsEnabled())
            {
                autoPaths_.periodic(yaw, swerveDrive_);
            }
            else
            {
                swerveDrive_->periodic(yaw, controls_);
            }

            if(frc::DriverStation::IsEnabled())
            {
                shooter_->periodic(-yaw);
                climb_.periodic(navx_->GetRoll());
                frc::SmartDashboard::PutNumber("Roll", navx_->GetRoll());
            }

        }, 5_ms, 2_ms);

}

void Robot::RobotInit()
{
    autoChooser_.SetDefaultOption("Taxi Dumb", AutoPaths::TAXI_DUMB);
    autoChooser_.AddOption("Dead Bot", AutoPaths::DEAD_BOT);
    autoChooser_.AddOption("Two Dumb", AutoPaths::TWO_DUMB);
    autoChooser_.AddOption("One Dumb Delayed", AutoPaths::ONE_DUMB_DELAYED);
    autoChooser_.AddOption("Straight Back", AutoPaths::STRAIGHT_BACK);
    autoChooser_.AddOption("Two Right", AutoPaths::TWO_RIGHT);
    autoChooser_.AddOption("Two Middle", AutoPaths::TWO_MIDDLE);
    autoChooser_.AddOption("Two Left", AutoPaths::TWO_LEFT);
    autoChooser_.AddOption("Three", AutoPaths::THREE);
    autoChooser_.AddOption("BIG BOY", AutoPaths::BIG_BOY);
    frc::SmartDashboard::PutData("Auto Modes", &autoChooser_);

    frc::SmartDashboard::PutNumber("Auto Yaw Offset", 0);

    controls_->setClimbMode(false);

    try
    {
        navx_ = new AHRS(frc::SPI::Port::kMXP);
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    navx_->ZeroYaw();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
    alliance_ = frc::DriverStation::GetAlliance();
    
    if(alliance_ == frc::DriverStation::Alliance::kBlue)
    {
        channel_->setColor(Channel::BLUE);
        frc::SmartDashboard::PutBoolean("Alliance Color", true);
    }
    else
    {
        channel_->setColor(Channel::RED);
        frc::SmartDashboard::PutBoolean("Alliance Color", false);
    }
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    shooter_->reset();
    climb_.setPneumatics(false, false);
    climb_.setState(Climb::MANUAL);
    climbTimer_.Stop();
    climbTimer_.Reset();
    climbTimer_.Start();

    AutoPaths::Path path = autoChooser_.GetSelected();
    //m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
    //fmt::print("Auto selected: {}\n", m_autoSelected);
    autoPaths_.setPath(path);

    navx_->ZeroYaw();
    yawOffset_ = autoPaths_.initYaw();
    

    swerveDrive_->reset();

    autoPaths_.startTimer();
}

void Robot::AutonomousPeriodic()
{
    limelight_->lightOn(true);
    if(climbTimer_.Get().value() < 0.2)
    {
        climb_.extendArms(-3);
    }
    else
    {
        climb_.stop();
    }

    Intake::State intakeState = autoPaths_.getIntakeState();
    Shooter::State shooterState = autoPaths_.getShooterState();

    if(channel_->isBallGood() || shooter_->getState() == Shooter::UNLOADING)
    {
        shooterState = Shooter::UNLOADING;
    }

    if((shooterState == Shooter::SHOOTING || shooterState == Shooter::UNLOADING) && intakeState != Intake::INTAKING)
    {
        intakeState = Intake::LOADING;
    }

    intake_.setState(autoPaths_.getIntakeState());
    shooter_->setState(autoPaths_.getShooterState());

    intake_.periodic();
}

void Robot::TeleopInit()
{
    controls_->setClimbMode(false);

    //odometryLogger_->openFile();
    //flywheelLogger_->openFile();
    //hoodLogger_->openFile();
    //turretLogger_->openFile();

    //frc::SmartDashboard::PutNumber("InV", 0);
    //frc::SmartDashboard::PutNumber("InA", 0);
    //frc::SmartDashboard::PutNumber("InHV", 0);
    //frc::SmartDashboard::PutNumber("fKp", 0);
    //frc::SmartDashboard::PutNumber("HINV", 0);
    //frc::SmartDashboard::PutNumber("FINV", 0);
    //frc::SmartDashboard::PutNumber("InDist", 4.0);
    //frc::SmartDashboard::PutNumber("K", 2);
    //frc::SmartDashboard::PutNumber("ITV", 0.0);
    //frc::SmartDashboard::PutNumber("InT", 0.0);
    //frc::SmartDashboard::PutNumber("smiv", 0.0);
    //frc::SmartDashboard::PutNumber("InCV", 0.0);
    //frc::SmartDashboard::PutNumber("Swerve Volts", 0.0);
    //frc::SmartDashboard::PutNumber("tkP", 0.0);
    //frc::SmartDashboard::PutNumber("tkI", 0.0);
    //frc::SmartDashboard::PutNumber("tkD", 0.0);

}

void Robot::TeleopPeriodic()
{
    controls_->periodic();
    frc::SmartDashboard::PutBoolean("Climb Mode", controls_->getClimbMode());

    if(controls_->fieldOrient())
    {
        navx_->ZeroYaw();
        yawOffset_ = 0;
    }

    if(!controls_->getClimbMode())
    {
        if(controls_->autoClimbCancelled()) 
        {
            shooter_->zeroHood();
        }

        if(controls_->autoClimbPressed() && !controls_->shootPressed()) //TODO reusing names again, change or something later
        {
            channel_->clearBalls();
            shooter_->clearBallShooting();
        }

        climb_.setState(Climb::DOWN);
        climb_.setAutoState(Climb::UNINITIATED);
    
        if(controls_->increaseRange())
        {
            shooter_->increaseRange();
        }
        if(controls_->decreaseRange())
        {
            shooter_->decreaseRange();
        }

        frc::SmartDashboard::PutBoolean("BAD IDEA", channel_->isBallGood());

        if((channel_->isBallGood() || shooter_->getState() == Shooter::UNLOADING) && !controls_->resetUnload())
        {
            shooter_->setState(Shooter::UNLOADING);
            intake_.setState(Intake::LOADING);
            //intake_.setState(Intake::INTAKING);
        }
        else if(controls_->shootPressed())
        {
            shooter_->setState(Shooter::SHOOTING);
            intake_.setState(Intake::LOADING);
            //intake_.setState(Intake::INTAKING);
        }
        else if(channel_->getBallCount() > 0)
        {
            shooter_->setState(Shooter::REVING);
        }
        else
        {
            shooter_->setState(Shooter::TRACKING);
            intake_.setState(Intake::RETRACTED_IDLE);
        }

        if (controls_->intakePressed())
        {
            if(shooter_->getState() != Shooter::SHOOTING)
            {
                shooter_->setState(Shooter::REVING);
            }
            intake_.setState(Intake::INTAKING);
        }
        else if (controls_->outakePressed())
        {
           intake_.setState(Intake::OUTAKING);
           shooter_->setState(Shooter::OUTAKING);
        }
        else if(intake_.getState() != Intake::LOADING)
        {
            intake_.setState(Intake::RETRACTED_IDLE);
            //intake_.setState(Intake::EXTENDED_IDLE);
        }
        /*else
        {
            intake_.setState(Intake::RETRACTED_IDLE);
        }*/

        if(controls_->manuallyOverrideTurret())
        {
            shooter_->setState(Shooter::MANUAL);
        }

    }
    else
    {
        intake_.setState(Intake::RETRACTED_IDLE);
        shooter_->setState(Shooter::CLIMB);

        if(climb_.getState() == Climb::IDLE || climb_.getState() == Climb::DOWN)
        {
            climb_.setState(Climb::MANUAL);
        }

        if(controls_->autoClimbPressed())
        {
            climb_.setState(Climb::AUTO);
            if(climb_.stageComplete())
            {
                climb_.readyNextStage();
            }
        }
        else if(controls_->autoClimbCancelled())
        {
            climb_.setState(Climb::MANUAL);
            climb_.setAutoState(Climb::UNINITIATED);
        }

        if(climb_.getState() == Climb::MANUAL)
        {
            if(controls_->getPneumatic1Toggle())
            {
                climb_.togglePneumatic1();
            }

            if(controls_->getPneumatic2Toggle())
            {
                climb_.togglePneumatic2();
            }

            climb_.extendArms(controls_->getClimbPower());
        }
        
    }
    
    /*stringstream odometry;
    odometry << swerveDrive_->getX() << ", " << swerveDrive_->getY() << ", " 
    << swerveDrive_->getSmoothX() << ", " << swerveDrive_->getSmoothY() << ", " 
    << swerveDrive_->getSWX() << ", " << swerveDrive_->getSWY();
    odometryLogger_->print(odometry.str());*/

    /*stringstream flywheel;
    flywheel << shooter_->getFlyVel();
    flywheelLogger_->print(flywheel.str());*/

    //stringstream hood;
    //hood << shooter_->getHoodTicks() << ", " << shooter_->getHoodVel() << ", " << shooter_->getHoodWantedVel();
    //hood << shooter_->getHoodTicks() << ", " << shooter_->getHoodVel();
    //hoodLogger_->addPrint(hood.str());

    //hoodLogger_->print(hood.str());
    //cout << shooter_->getHoodWantedVel() << ", " << shooter_->getHoodVel() * 10 << endl;
    //hoodLogger_->print(shooter_->getHoodWantedVel());

    /*stringstream turret;
    turret << shooter_->getTurretAngle();
    turretLogger_->print(turret.str());*/

    //frc::SmartDashboard::PutNumber("yaw", navx_->GetYaw());

    /*double yaw = navx_->GetYaw() - yawOffset_;
    Helpers::normalizeAngle(yaw);

    swerveDrive_->periodic(yaw, controls_);
    shooter_->periodic(-yaw);
    climb_.periodic(navx_->GetRoll());*/

    shooter_->setTurretManualVolts(controls_->getTurretManual());
    
    intake_.periodic();
}

void Robot::DisabledInit()
{
    //COMP Disable from here
    //shooter_->reset();
    limelight_->lightOn(true); //Not here

    shooter_->setState(Shooter::IDLE); //Not here
    //shooter_->periodic(-navx_->GetYaw());

    //swerveDrive_->reset(); //COMP Disable to here

    autoPaths_.setSetPath(false);

    //odometryLogger_->closeFile();
    //flywheelLogger_->closeFile();
    //hoodLogger_->closeFile();
    //turretLogger_->closeFile();

    //hoodLogger_->print();
}

void Robot::DisabledPeriodic()
{
    //shooter_->reset(); //COMP Disable this
    limelight_->lightOn(true);

    swerveDrive_->reset();

    autoPaths_.setSetPath(false);
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
