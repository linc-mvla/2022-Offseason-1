#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "TrajectoryCalc.h"

class Hood
{
    public:
        bool party(){return true;}
        enum State
        {
            IDLE, 
            IMMOBILE,
            AIMING,
            ZEROING,
        };
        State getState();
        void setState(State state);
        void setPID(double p, double i, double d);
        double getHoodTicks();
        double getHoodVel();
        double getHoodWantedVel();

        bool isReady();

        Hood();
        void periodic();

        void reset();
        void zero();
        void setWantedPos(double setPos);
        void move();

        void setInVolts(double inVolts);
        
        double calcPID();
        double angleToTicks(double angle);
        //void resetPID();

    private:
        WPI_TalonFX hoodMotor_;

        double maxV = 100000;
        double maxA = 500000 * 1.5;
        double kP = 0.0001;
        double kD = 0;
        double kV = 1 / ShooterConstants::HOOD_NEG_FF;
        double kA = 0;
        double kVI = ShooterConstants::HOOD_NEG_FF_INTERCEPT;
        TrajectoryCalc trajectoryCalc_;
        bool initTrajectory_;
        double setTrajectoryPos_;

        State state_;
        bool zeroed_, currentStopHit_;
        double setPos_, prevError_, integralError_;
        double kP_ = 0.0008; //TODO tune values
        bool kI_ = false; //IMPORTANT will brek everything like the tf2 coconut
        double kD_ = 0.0;

        double prevTime_, dT_;
        frc::Timer timer_;
        //0.0008, 0.000, 0.0000001
        //0.0025, 0.0001, 0.0000001

        double inVolts_;
};