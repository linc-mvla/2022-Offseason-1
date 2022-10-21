#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"
#include "Helpers.h"
#include "SwerveDrive.h"

class Turret
{
    public:
        enum State
        {
            IDLE,
            IMMOBILE,
            TRACKING,
            UNLOADING,
            MANUAL,
            CLIMB
        };
        State getState();
        void setState(State state);
        void setManualVolts(double manualVolts);
        bool isAimed();
        bool unloadReady();
        double getAngle();

        Turret(Limelight* limelight, SwerveDrive* swerveDrive);
        void periodic(double yaw, double offset);
        void reset();

        void track();
        void calcUnloadAng();

        double calcAngularFF();
        double calcLinearFF();
        double calcError();
        double calcPID();
        double calcProfileVolts();

        void setInPos(double pos);
        
    private:
        WPI_TalonFX turretMotor_;
        Limelight* limelight_;
        SwerveDrive* swerveDrive_;

        double maxV = 360;
        double maxA = 3600;
        double kP = 0;
        double kD = 0;
        double kV = 1 / ShooterConstants::TURRET_FF;
        double kVI = ShooterConstants::TURRET_FF_INTERCEPT;
        double kA = 0;
        TrajectoryCalc trajectoryCalc_;
        bool initTrajectory_;

        State state_;
        double manualVolts_;
        bool aimed_, unloadReady_;
        double prevYaw_, yaw_, offset_, unloadAngle_;

        double prevTime_, dT_, deltaYaw_, yawDT_, yawPrevTime_, yawVel_, currentSetPos_;
        frc::Timer timer_;

        double prevError_, integralError_;
        double tkP_ = 0.25; //TODO tune, tuned with physics only, could be more aggressive
        double tkI_ = 0.000/*1*/;  //0.075, 0.0001, 0.000008
        double tkD_ = 0.00001;  //0.3, 0.0001, 0.00005
        //0.2, 0.0001, 0.000008

        double inPos_;
};