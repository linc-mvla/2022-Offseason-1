#include "Hood.h"

Hood::Hood() : hoodMotor_(ShooterConstants::HOOD_ID), trajectoryCalc_(maxV, maxA, kP, kD, kV, kA, kVI)
{
    hoodMotor_.SetNeutralMode(NeutralMode::Coast);
    reset();

    state_ = IDLE;
}

Hood::State Hood::getState()
{
    return state_;
}

void Hood::setState(State state)
{
    if(state == ZEROING)
    {
        zeroed_ = false;
        currentStopHit_ = false;
    }
    state_ = state;
}

void Hood::setPID(double p, double i, double d)
{
    kP_ = p;
    kI_ = i;
    kD_ = d;
}

double Hood::getHoodTicks()
{
    return hoodMotor_.GetSelectedSensorPosition();
}

double Hood::getHoodVel()
{
    return hoodMotor_.GetSelectedSensorVelocity();
}

double Hood::getHoodWantedVel()
{
    return get<1>(trajectoryCalc_.getProfile());
}

bool Hood::isReady()
{
    frc::SmartDashboard::PutNumber("Herror", abs(setPos_ - hoodMotor_.GetSelectedSensorPosition()));
    return (abs(setPos_ - hoodMotor_.GetSelectedSensorPosition()) < ShooterConstants::HOOD_READY);
}

void Hood::periodic()
{
    if(hoodMotor_.GetSelectedSensorPosition() < ShooterConstants::MAX_HOOD_TICKS - 100)
    {
        zeroed_ = false;
    }


    if(!zeroed_)
    {
        state_ = ZEROING;
        //zeroed_ = false;
    }

    switch(state_)
    {
        case IDLE:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Coast);
            hoodMotor_.SetVoltage(units::volt_t(0));
            break;
        }
        case IMMOBILE:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Brake);
            hoodMotor_.SetVoltage(units::volt_t(0));
            break;
        }
        case AIMING:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Brake);
            move();
            break;
        }
        case ZEROING:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Brake);
            zero();
            break;
        }
    }
}

void Hood::reset()
{
    zeroed_ = party();
    currentStopHit_ = false;
    hoodMotor_.SetSelectedSensorPosition(0);
    initTrajectory_ = false;
}

void Hood::zero()
{
    //frc::SmartDashboard::PutNumber("HCUR", hoodMotor_.GetSupplyCurrent());
    //frc::SmartDashboard::PutNumber("HOOD ZVEL", hoodMotor_.GetSelectedSensorVelocity());
    //frc::SmartDashboard::PutNumber("HOOD ZPOS", hoodMotor_.GetSelectedSensorPosition());
    if(/*hoodMotor_.GetSelectedSensorVelocity() < 20*/hoodMotor_.GetSupplyCurrent() > ShooterConstants::HOOD_ZERO_CURRENT)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
        currentStopHit_ = true;
        hoodMotor_.SetSelectedSensorPosition(0);
        //hoodMotor_.SetSelectedSensorPosition(230); //TODO change to a constant
        //zeroed_ = true;
        //state_ = IMMOBILE;
    }

    if(currentStopHit_)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));

        if(abs(hoodMotor_.GetSelectedSensorVelocity()) < 5 && hoodMotor_.GetSelectedSensorPosition() < -10)
        {
            hoodMotor_.SetSelectedSensorPosition(0);
            currentStopHit_ = false;
            zeroed_ = true;
            state_ = IMMOBILE;
        }
    }
    else
    {
        hoodMotor_.SetVoltage(units::volt_t(1)); //TODO change to constant
    }
}

void Hood::setWantedPos(double setPos)
{
    //setPos_ = setPos;
    setPos_ = angleToTicks(setPos);
}

void Hood::move()
{
    //double volts = calcPID();

    //volts = inVolts_;

    //volts = frc::SmartDashboard::GetNumber("HINV", 0);
    //setPos_ = frc::SmartDashboard::GetNumber("InA", 0);

    double volts;
    if(abs(setPos_ - setTrajectoryPos_) > 50 && initTrajectory_) //TODO get value
    {
        setTrajectoryPos_ = setPos_;
        //double pos = get<2>(trajectoryCalc_.getProfile());
        //double vel = get<1>(trajectoryCalc_.getProfile());
        double pos = hoodMotor_.GetSelectedSensorPosition();
        double vel = hoodMotor_.GetSelectedSensorVelocity() * 10;

        trajectoryCalc_.generateTrajectory(pos, setPos_, vel);
    }
    else if(!initTrajectory_)
    {
        initTrajectory_ = true;
        setTrajectoryPos_ = setPos_;

        double pos = hoodMotor_.GetSelectedSensorPosition();
        double vel = hoodMotor_.GetSelectedSensorVelocity() * 10;

        trajectoryCalc_.generateTrajectory(pos, setPos_, vel);
    }

    if(initTrajectory_)
    {
        double pos = hoodMotor_.GetSelectedSensorPosition();
        double vel = hoodMotor_.GetSelectedSensorVelocity() * 10;
        //volts = trajectoryCalc_.calcPower(pos, vel)// + ShooterConstants::HOOD_WEIGHT_FF;

        tuple<double, double, double> profile = trajectoryCalc_.getProfile();

        double profileVel = get<1>(profile);
        double profileAcc = get<0>(profile);
        double profilePos = get<2>(profile);
        double kVVolts;
        if(profileVel < 0)
        {
            kVVolts = (profileVel - ShooterConstants::HOOD_NEG_FF_INTERCEPT) / ShooterConstants::HOOD_NEG_FF;
        }
        else if(profileVel > 0)
        {
            kVVolts = (profileVel - ShooterConstants::HOOD_POS_FF_INTERCEPT) / ShooterConstants::HOOD_POS_FF;
        }
        else
        {
            //kVVolts = 0;
            kVVolts = ShooterConstants::HOOD_WEIGHT_FF;
        }

        if(profileVel == 0 && profileAcc == 0/* && vel < 10*/)
        {
            volts = ((setPos_ - pos) * kP_) + ShooterConstants::HOOD_WEIGHT_FF;
            //volts = ((profilePos - pos) * kP_) + ShooterConstants::HOOD_WEIGHT_FF;
        }
        else
        {
            volts = ((get<2>(profile) - pos) * kP) + ((profileVel - vel) * kD) + kVVolts + (profileAcc * kA);
        }
        

    }
    else
    {
        volts = 0;
    }
    
    frc::SmartDashboard::PutNumber("Ang Ticks", hoodMotor_.GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber("HVEL", hoodMotor_.GetSelectedSensorVelocity());
    
    //0, 100-120
    //0.5, 540
    //1, 1250
    //1.5, 2150
    //2, 3060
    //2.5, 3600


    if(hoodMotor_.GetSelectedSensorPosition() < ShooterConstants::MAX_HOOD_TICKS && volts < 0)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
    }
    else if(hoodMotor_.GetSelectedSensorPosition() > 0 && volts > 0)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
    }
    else
    {
        hoodMotor_.SetVoltage(units::volt_t(volts));
    }
    //hoodMotor_.SetVoltage(units::volt_t(volts));
    
    //cout << volts << endl;
    //frc::SmartDashboard::PutNumber("HV", volts);
}

void Hood::setInVolts(double inVolts)
{
    inVolts_ = inVolts;
}

double Hood::calcPID()
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    double error = setPos_ - hoodMotor_.GetSelectedSensorPosition();

    integralError_ += error * dT_;
    double deltaError = (error - prevError_) / dT_;
    if(abs(prevError_) < 50.0 && abs(error > 125)) //TODO get value, probably same as above
    {
        deltaError = 0;
        integralError_ = 0;
    } 
    prevError_ = error;

    double power = (kP_*error) + (kI_*integralError_) + (kD_*deltaError);
    power += ShooterConstants::HOOD_WEIGHT_FF;

    //frc::SmartDashboard::PutNumber("Herror", error);
    //frc::SmartDashboard::PutNumber("HP", power);
    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3);
}

double Hood::angleToTicks(double angle)
{
    return (angle - ShooterConstants::MAX_HOOD_ANGLE) * ShooterConstants::TICKS_PER_HOOD_DEGREE;
}

//-1, -8200
//-2, -28500
//-3, -46900
//-4, -65600
//-5, -83980
//-6, -102900

//-1, -5100
//-2, -23000
//-3, -42000
//-4, -60000