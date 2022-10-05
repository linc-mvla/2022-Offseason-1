#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "TrajectoryCalc.h"

#include <frc/Solenoid.h>

#include <numeric>

class Climb
{
    public:
        enum State
        {
            IDLE,
            DOWN,
            AUTO,
            MANUAL
        };

        enum AutoState
        {
            UNINITIATED,
            CLIMB_LOW,
            EXTEND_TO_MID,
            CLIMB_MID, 
            EXTEND_TO_HIGH,
            CLIMB_HIGH,
            DONE
        };

    State getState();
    void setState(State state);

    AutoState getAutoState();
    void setAutoState(AutoState autoState);

    Climb();
    void periodic(double pitch);

    void setPneumatics(bool pneumatic1, bool pneumatic2);
    void togglePneumatic1();
    void togglePneumatic2();
    void extendArms(double power);
    void stop();
    void autoClimb();
    bool stageComplete();
    void readyNextStage();

    bool climbBar();
    bool raiseToBar();

    void setBrake(bool brake);

    private:

        WPI_TalonFX gearboxMaster_;
        WPI_TalonFX gearboxSlave_;

        frc::Solenoid pneumatic1_;
        frc::Solenoid pneumatic2_;

        frc::Solenoid brake_;

        double bottomPos_, startTime_, midCurrent_;
        vector<double> climbCurrents_;
        bool waiting_;
        frc::Timer timer_;

        double maxV = 100000;
        double maxA = 1000000 * 2;
        double kP = 0;
        double kD = 0;
        double kV = 1 / ClimbConstants::RAISE_FF;
        double kVI = ClimbConstants::RAISE_FF_INTERCEPT;
        double kA = 0;
        TrajectoryCalc trajectoryCalc_;
        bool initTrajectory_;

        State state_;
        AutoState autoState_;

        double pitch_;
        bool nextStage_, stageComplete_;

        double kP_ = 0.0001;

};

//138207
//15821
//0.59
//207433.898

//147158
//35373
//0.54
//207009.259

//111800
//27979
//0.41
//204441.463

//83971
//-7331
//0.43
//212330.233


//110172
//45052
//0.62
//105032.258

//65285
//-6998
//0.63
//114734.921

//105620
//8337
//0.92
//105742.391

//17623.7319
//17400 ticks/sec/volt