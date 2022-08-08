#include "Climb.h"

Climb::Climb() : gearboxMaster_(ClimbConstants::MASTER_ID), gearboxSlave_(ClimbConstants::SLAVE_ID), 
pneumatic1_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_1_ID), 
pneumatic2_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_2_ID), 
brake_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::BRAKE_ID), trajectoryCalc_(maxV, maxA, kP, kD, kV, kA, kVI)
{
    gearboxMaster_.SetNeutralMode(NeutralMode::Brake);
    gearboxSlave_.SetNeutralMode(NeutralMode::Brake);

    gearboxSlave_.Follow(gearboxMaster_);

    state_ = IDLE;
    autoState_ = UNINITIATED;

}

Climb::State Climb::getState()
{
    return state_;
}

void Climb::setState(State state)
{
    state_ = state;
}

Climb::AutoState Climb::getAutoState()
{
    return autoState_;
}

void Climb::setAutoState(AutoState autoState)
{
    autoState_ = autoState;
}

void Climb::periodic(double roll)
{
    roll_ = roll;
    //frc::SmartDashboard::PutNumber("Roll", roll);
    double pos = gearboxMaster_.GetSelectedSensorPosition();
    //frc::SmartDashboard::PutNumber("ClimbPos", pos);
    //frc::SmartDashboard::PutNumber("CV", gearboxMaster_.GetSelectedSensorVelocity());

    switch(state_)
    {
        case IDLE:
        {
            stop();
            setBrake(true);
            break;
        }
        case DOWN:
        {
            setBrake(true);
            setPneumatics(false, false);
            stop();
            break;
        }
        case AUTO:
        {
            autoClimb();
            break;
        }
        case MANUAL:
        {
            setBrake(false);
            break;
        }
    }
}

void Climb::setPneumatics(bool pneumatic1, bool pneumatic2)
{
    pneumatic1_.Set(!pneumatic1);
    pneumatic2_.Set(pneumatic2);
}

void Climb::togglePneumatic1()
{
    pneumatic1_.Toggle();
}

void Climb::togglePneumatic2()
{
    pneumatic2_.Toggle();
}

void Climb::extendArms(double power)
{
    //frc::SmartDashboard::PutNumber("CP", gearboxMaster_.GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber("CC", gearboxMaster_.GetSupplyCurrent());
    gearboxMaster_.SetVoltage(units::volt_t(power)); //TODO check if slave motor spins as well
}

void Climb::stop()
{
    gearboxMaster_.SetVoltage(units::volt_t(0));
}

void Climb::autoClimb()
{
    if(autoState_ != DONE)
    {
        setBrake(false);
    }

    readyNextStage();

    switch(autoState_)//TODO clean up switch? Kinda redundant
    {
        case UNINITIATED:
        {
            gearboxMaster_.SetSelectedSensorPosition(0);
            climbCurrents_.clear();
            setBrake(false);
            stageComplete_ = false;
            nextStage_ = false;
            initTrajectory_ = false;
            waiting_ = false;
            autoState_ = CLIMB_LOW;
            break;
        }
        case CLIMB_LOW:
        {
            if(!stageComplete_)
            {
                if(climbBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                initTrajectory_ = false;
                waiting_ = false;
                autoState_ = EXTEND_TO_MID;
            }
            break;
        }
        case EXTEND_TO_MID:
        {
            if(!stageComplete_)
            {
                if(raiseToBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                initTrajectory_ = false;
                waiting_ = false;
                autoState_ = CLIMB_MID;
            }
            break;
        }
        case CLIMB_MID:
        {
            if(!stageComplete_)
            {
                if(climbBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                initTrajectory_ = false;
                waiting_ = false;
                autoState_ = EXTEND_TO_HIGH;
            }
            break;
        }
        case EXTEND_TO_HIGH:
        {
            if(!stageComplete_)
            {
                if(raiseToBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                initTrajectory_ = false;
                waiting_ = false;
                autoState_ = CLIMB_HIGH;
            }
            break;
        }
        case CLIMB_HIGH: //TODO, some way of figuring out when it's done other than manual cancel or all the way up
        {
            if(!stageComplete_)
            {
                if(climbBar())
                {
                    stageComplete_ = true;
                }
            }
            else
            {
                autoState_ = DONE;
            }
            break;
        }
        case DONE:
        {
            gearboxMaster_.SetVoltage(units::volt_t(0));
            setBrake(true);
            setPneumatics(true, true);
            break;
        }
    }

    
}

bool Climb::stageComplete()
{
    return stageComplete_;
}

void Climb::readyNextStage()
{
    nextStage_ = true;
}

bool Climb::climbBar()
{
    //frc::SmartDashboard::PutNumber("CC", gearboxMaster_.GetSupplyCurrent());
    
    //double currentSpike = 0;
    double volts = 0;
    if(autoState_ == CLIMB_LOW)
    {
        //currentSpike = ClimbConstants::LOW_STALL_CURRENT_SPIKE;
        volts = ClimbConstants::LOW_CLIMB_VOLTAGE;
    }
    else if(autoState_ == CLIMB_MID)
    {
        //currentSpike = ClimbConstants::MID_STALL_CURRENT_SPIKE;
        volts = ClimbConstants::MID_CLIMB_VOLTAGE;
    }
    else if(autoState_ == CLIMB_HIGH)
    {
        //currentSpike = ClimbConstants::HIGH_STALL_CURRENT_SPIKE;
        volts = ClimbConstants::HIGH_CLIMB_VOLTAGE;
    }

    /*if(abs(gearboxMaster_.GetSelectedSensorPosition() - ClimbConstants::CLEAR_OF_BARS) < 5000)
    {
        climbCurrents_.push_back(gearboxMaster_.GetSupplyCurrent());
        cout << gearboxMaster_.GetSupplyCurrent() << endl;
        midCurrent_ = accumulate(climbCurrents_.begin(), climbCurrents_.end(), 0.0) / climbCurrents_.size();
    }*/

    //frc::SmartDashboard::PutNumber("MID CUR", midCurrent_);
    //frc::SmartDashboard::PutNumber("CLIMB VEL", gearboxMaster_.GetSelectedSensorVelocity());

    if(/*gearboxMaster_.GetSupplyCurrent() > (midCurrent_ + currentSpike)*/ abs(gearboxMaster_.GetSelectedSensorVelocity()) < 100 && gearboxMaster_.GetSelectedSensorPosition() > ClimbConstants::NEARING_HARDSTOP)
    {
        gearboxMaster_.SetVoltage(units::volt_t(0));
        bottomPos_ = gearboxMaster_.GetSelectedSensorPosition();
        climbCurrents_.clear();
        return true;
    }

    if(autoState_ == CLIMB_HIGH && gearboxMaster_.GetSelectedSensorPosition() > ClimbConstants::CLEAR_OF_BARS)
    {
        gearboxMaster_.SetVoltage(units::volt_t(0));
        return true;
    }

    if(gearboxMaster_.GetSelectedSensorPosition() > ClimbConstants::NEARING_HARDSTOP)
    {
        gearboxMaster_.SetVoltage(units::volt_t(ClimbConstants::SLOW_CLIMB_VOLTAGE));
    }
    else
    {
        gearboxMaster_.SetVoltage(units::volt_t(volts));
    }

    
    return false;
}



bool Climb::raiseToBar()
{
    double pos = gearboxMaster_.GetSelectedSensorPosition();
    frc::SmartDashboard::PutNumber("CP", pos);
    //double posFromBottom = bottomPos_ - pos;
    //frc::SmartDashboard::PutNumber("CPFB", posFromBottom);
    //18442
    //22201

    trajectoryCalc_.setPrintError(true);
    
    if(pos > bottomPos_ - ClimbConstants::TOO_FAR_FROM_STATIC_HOOKS)
    {
        //frc::SmartDashboard::PutBoolean("RAISING", false);
        gearboxMaster_.SetVoltage(units::volt_t(ClimbConstants::SUPER_SLOW_RAISE_VOLTAGE));
    }
    else if(pos > bottomPos_ - ClimbConstants::ABOVE_STATIC_HOOKS)
    {
        //frc::SmartDashboard::PutBoolean("RAISING", true);
        gearboxMaster_.SetVoltage(units::volt_t(ClimbConstants::SLOW_RAISE_VOLTAGE));
    }
    else
    {
        if(!initTrajectory_)
        {
            initTrajectory_ = true;
            trajectoryCalc_.generateTrajectory(gearboxMaster_.GetSelectedSensorPosition(), 0, gearboxMaster_.GetSelectedSensorVelocity());
        }

        gearboxMaster_.SetVoltage(units::volt_t(trajectoryCalc_.calcPower(gearboxMaster_.GetSelectedSensorPosition(), gearboxMaster_.GetSelectedSensorVelocity())));
    }

    if(autoState_ == EXTEND_TO_MID)
    {
        if(abs(pos) < ClimbConstants::EXTEND_THRESHOLD || waiting_) //TODO make more strict, maybe when motion is slow?
        {
            setPneumatics(true, true);
            if(!waiting_)
            {
                startTime_ = timer_.GetFPGATimestamp().value();
                waiting_ = true;
            }

            //double time = timer_.GetFPGATimestamp().value() - startTime_;
            //frc::SmartDashboard::PutNumber("time", time);
            return (timer_.GetFPGATimestamp().value() - startTime_ > ClimbConstants::ON_BAR_DELAY);
        }

        if(pos < ClimbConstants::CLEAR_OF_BARS && pos > ClimbConstants::EXTEND_THRESHOLD)
        {
            setPneumatics(true, false);
        }
    }
    else if(autoState_ == EXTEND_TO_HIGH)
    {
        //frc::SmartDashboard::PutNumber("CPOS", abs(pos));
        if(waiting_ || (abs(pos) < ClimbConstants::HIGH_EXTEND_THRESHOLD && (roll_ > ClimbConstants::ROLL_MAX || roll_ < ClimbConstants::ROLL_MIN)))
        {
            setPneumatics(true, false);
            if(!waiting_)
            {
                startTime_ = timer_.GetFPGATimestamp().value();
                waiting_ = true;
            }

            //double time = timer_.GetFPGATimestamp().value() - startTime_;
            //frc::SmartDashboard::PutNumber("time", time);
            return (timer_.GetFPGATimestamp().value() - startTime_ > ClimbConstants::ON_BAR_DELAY);
        }
    }

    return false;
}

void Climb::setBrake(bool brake)
{
    brake_.Set(!brake);
}

//123905
//116580
//110000

//-0.7, -380
//-1, -1060
//-2, -2990
//-3, -4900
//-4, -6810
//-5, -8730
//-6, -10640
//-7, -12600
//-8, -14530
//-9, -16500
//-10, -18440
//-11, -20400
//-12, -22170