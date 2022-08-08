#include "Turret.h"

Turret::Turret(Limelight* limelight, SwerveDrive* swerveDrive) : turretMotor_(ShooterConstants::TURRET_ID), trajectoryCalc_(maxV, maxA, kP, kD, kV, kA, kVI)
{
    turretMotor_.SetNeutralMode(NeutralMode::Coast);
    reset();
    limelight_ = limelight;
    swerveDrive_ = swerveDrive;
    currentSetPos_ = 0;

    state_ = IDLE;
}

void Turret::periodic(double yaw, double offset)
{
    yaw_ = yaw;
    offset_ = offset;
    //robotGoalAng_ = robotGoalAng;
    //foundGoal_ = foundGoal;
    //x_ = x;
    //y_ = y;

    switch(state_) //TODO clean up/combine casees after the switch
    {
        case IDLE:
        {
            turretMotor_.SetVoltage(units::volt_t(0));
            turretMotor_.SetNeutralMode(NeutralMode::Coast);
            break;
        }
        case IMMOBILE:
        {
            turretMotor_.SetVoltage(units::volt_t(0));
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            break;
        }
        case TRACKING:
        {
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            track();
            break;
        }
        case UNLOADING:
        {
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            calcUnloadAng();
            track();
            break;
        }
        case MANUAL:
        {
            turretMotor_.SetVoltage(units::volt_t(manualVolts_));
            frc::SmartDashboard::PutNumber("TX", limelight_->getAdjustedX());
            //frc::SmartDashboard::PutNumber("TANG", getAngle());
            //calcError(); //TODO remove, just for printing values
            limelight_->lightOn(true);
            //frc::SmartDashboard::PutNumber("TV", turretMotor_.GetSelectedSensorVelocity());
            break;
        }
        case CLIMB:
        {
            limelight_->lightOn(false);
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            track();
            break;
        }
    }
}

Turret::State Turret::getState()
{
    return state_;
}

void Turret::setState(State state)
{
    state_ = state;
}

void Turret::setManualVolts(double manualVolts)
{
    manualVolts_ = manualVolts;
}

bool Turret::isAimed()
{
    //return true;
    return aimed_;
}

bool Turret::unloadReady()
{
    //return true;
    return unloadReady_;
}

double Turret::getAngle()
{
    return turretMotor_.GetSelectedSensorPosition() / ShooterConstants::TICKS_PER_TURRET_DEGREE;
}

void Turret::reset()
{
    turretMotor_.SetSelectedSensorPosition(0);
    initTrajectory_ = false;
}

void Turret::track()
{
    frc::SmartDashboard::PutBoolean("Found goal", swerveDrive_->foundGoal());
    if(!limelight_->hasTarget() && !swerveDrive_->foundGoal()) //REMOVE THIS PLEASE
    {
        turretMotor_.SetVoltage(units::volt_t(0));
        limelight_->lightOn(true);
    }
    else
    {
        double volts = calcPID();

        double time = timer_.GetFPGATimestamp().value();
        dT_ = time - prevTime_;
        yawDT_ = time - yawPrevTime_;

        prevTime_ = time;

        deltaYaw_ = yaw_ - prevYaw_;
        //cout << "yaws: " << yaw_ << ", " << prevYaw_ << ", " << deltaYaw_ << "," << yawVel_ << endl;

        if(deltaYaw_ != 0)
        {
            prevYaw_ = yaw_;

            if(abs(deltaYaw_) > 300)
            {
                deltaYaw_ = (deltaYaw_ > 0) ? deltaYaw_ - 360 : deltaYaw_ + 360;
            }

            yawVel_ = deltaYaw_ / yawDT_;
            yawPrevTime_ = time;
        }

        /*double volts;
        double error = calcError();

        double pos = getAngle();
        frc::SmartDashboard::PutNumber("TP", pos);

        double vel = turretMotor_.GetSelectedSensorVelocity() * 10 / ShooterConstants::TICKS_PER_TURRET_DEGREE;
        frc::SmartDashboard::PutNumber("TV", vel);
        
        double setPos = pos + error;
        frc::SmartDashboard::PutNumber("TSP", setPos);
        if(abs(setPos > 180))
        {
            frc::SmartDashboard::PutNumber("SPTHING DIED", setPos);
            frc::SmartDashboard::PutNumber("ETHING DIED", error);
            frc::SmartDashboard::PutBoolean("THING DIED", true);
            return;
        }
        //frc::SmartDashboard::PutNumber("TSP", setPos);

        //double inPos = frc::SmartDashboard::GetNumber("InT", getAngle());
        //Helpers::normalizeAngle(inPos);
        //error = inPos - getAngle();
        //setPos = inPos;
        //frc::SmartDashboard::PutNumber("TE", error);

        //error = inPos_ - getAngle();
        //setPos = inPos_;
        //frc::SmartDashboard::PutNumber("TIP", inPos_);
        //frc::SmartDashboard::PutNumber("TE", error);
        //frc::SmartDashboard::PutBoolean("TR", (abs(error) < ShooterConstants::TURRET_AIMED));
        
        //initTrajectory_ = false;
        double wantedVel, wantedPos; 
        if(initTrajectory_ && abs(setPos - currentSetPos_) < 3)
        {
            double acc = get<0>(trajectoryCalc_.getProfile());
            wantedVel = get<1>(trajectoryCalc_.getProfile());
            if(abs(error) > ShooterConstants::TURRET_AIMED && wantedVel == 0 && acc == 0 && abs(vel) < 1)
            {
                trajectoryCalc_.generateTrajectory(pos, setPos, vel);
            }
            //else
            //{
                //wantedPos = get<2>(trajectoryCalc.getProfile());
                //trajectoryCalc_.generateTrajectory(wantedPos, setPos, wantedVel);
            //}
        }
        else
        {
            initTrajectory_ = true;
            currentSetPos_ = setPos;

            trajectoryCalc_.generateTrajectory(pos, setPos, vel);
            //wantedPos = get<2>(trajectoryCalc_.getProfile());
            //wantedVel = get<1>(trajectoryCalc_.getProfile());
        }

        //TODO clean all the stuff above up... like man

        //wantedPos = get<2>(trajectoryCalc_.getProfile());
        wantedVel = get<1>(trajectoryCalc_.getProfile());

        volts = trajectoryCalc_.calcPower(pos, vel);

        if(state_ != CLIMB && abs(getAngle()) < 178)
        {
            //volts += calcAngularFF();
            //volts += calcLinearFF();
        }*/

        //frc::SmartDashboard::PutNumber("T Volts", volts);
        if(volts > 0 && getAngle() > 175)
        {
            std::cout << "trying to decapitate itself" << std::endl;
            turretMotor_.SetVoltage(units::volt_t(0));
        }
        else if (volts < 0 && getAngle() < -175)
        {
            std::cout << "trying to decapitate itself" << std::endl;
            turretMotor_.SetVoltage(units::volt_t(0));
        }
        else
        {
            turretMotor_.SetVoltage(units::volt_t(volts));
        }
        
    }
    
}

void Turret::calcUnloadAng()
{
    double angToHangar = 0;
    double x = swerveDrive_->getX();
    double y = swerveDrive_->getY();

    double xDist = x - GeneralConstants::HANGAR_X;
    double yDist = y - GeneralConstants::HANGAR_Y;
    if(xDist != 0 || yDist != 0)
    {
        angToHangar = -(atan2(yDist, xDist) * 180 / M_PI) - 90;
    }

    double angToGoal = 0;
    if(x != 0 || y != 0)
    {
        angToGoal = -(atan2(y, x) * 180 / M_PI) - 90;
    }

    angToHangar += 360 * 10;
    angToHangar = ((int)floor(angToHangar) % 360) + (angToHangar - floor(angToHangar));

    angToGoal += 360 * 10; 
    angToGoal = ((int)floor(angToGoal) % 360) + (angToGoal - floor(angToGoal));

    if(abs(angToHangar - angToGoal) < 10) //TODO get value
    {
        angToHangar += (angToHangar > angToGoal) ? 10 : -10; //TODO incorporate goal diameter?
    }
    //Helpers::normalizeAngle(angToHangar);

    unloadAngle_ = 180 + angToHangar - yaw_;
    Helpers::normalizeAngle(unloadAngle_);
}

double Turret::calcAngularFF()
{
    //frc::SmartDashboard::PutNumber("PYAW", prevYaw_);
    double rff = -yawVel_ / 82;

    if(abs(yawVel_) < 5)
    {
        return 0;
    }

    double ff = -(abs(yawVel_) - ShooterConstants::TURRET_FF_INTERCEPT) / ShooterConstants::TURRET_FF;
    if(yawVel_ < 0)
    {
        ff *= -1;
    }

    //double radPerSec = -(yawVel_ * ShooterConstants::TICKS_PER_TURRET_DEGREE * 2 * M_PI) / GeneralConstants::TICKS_PER_ROTATION;
    //double rff = radPerSec / GeneralConstants::Kv; //TODO tune

    //frc::SmartDashboard::PutNumber("YAW3", yaw_);
    //frc::SmartDashboard::PutNumber("DYAW", deltaYaw_);
    //frc::SmartDashboard::PutNumber("TFF", ff);
    //frc::SmartDashboard::PutNumber("{TFF", rff);
    //return 0;

    return ff;
}

double Turret::calcLinearFF()
{
    double vel = swerveDrive_->getGoalXVel();
    double x = swerveDrive_->getX();
    double y = swerveDrive_->getY();
    double distance = sqrt(x * x + y * y);

    //double radPerSec = (-vel / distance) * 360 * ShooterConstants::TICKS_PER_TURRET_DEGREE / GeneralConstants::TICKS_PER_ROTATION;

    double degPerSec = (-vel / distance) * 180 / M_PI;
    if(abs(degPerSec) < 5)
    {
        return 0;
    }

    double ff = (abs(degPerSec) - ShooterConstants::TURRET_FF_INTERCEPT) / ShooterConstants::TURRET_FF;
    if(degPerSec < 0)
    {
        ff *= -1;
    }

    return ff * 0.5;
    //return radPerSec / GeneralConstants::Kv; //TODO tune
}

double Turret::calcError()
{
    if(!swerveDrive_->foundGoal())
    {
        return 0;
    }
    double error, goalError;
    if(state_ == UNLOADING)
    {
        error = unloadAngle_ - getAngle();
    }
    else if(state_ == CLIMB)
    {
        error = -getAngle();
        return error;
    }
    else if(limelight_->hasTarget())
    {
        error = offset_ + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET;
    }
    else
    {
        double wantedTurretAng = (180 - swerveDrive_->getRobotGoalAng()) + offset_;
        Helpers::normalizeAngle(wantedTurretAng);

        error = wantedTurretAng - getAngle();
    }

    if(limelight_->hasTarget())
    {
        goalError = limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET;
    }
    else
    {
        double wantedGoalAng = (180 - swerveDrive_->getRobotGoalAng());
        Helpers::normalizeAngle(wantedGoalAng);

        goalError = wantedGoalAng - getAngle();
    }


    //frc::SmartDashboard::PutNumber("TA", getAngle());
    frc::SmartDashboard::PutNumber("Terror", error);

    if(abs(error + getAngle()) > 180)
    {
        if(getAngle() > 0 && error > 0)
        {
            error -= 360;
        }

        else if(getAngle() < 0 && error < 0)
        {
            error += 0;
        }

        //error = (error > 0) ? error - 360 : error + 360;
    }

    if(abs(goalError) > 40 && !limelight_->hasTarget()) //COMP disable probably
    {
        limelight_->lightOn(false);
    }
    else if(state_ != CLIMB)
    {
        limelight_->lightOn(true);
    }

    aimed_ = (abs(error) < ShooterConstants::TURRET_AIMED); //TODO get value, change back to 2.5
    unloadReady_ = (abs(error) < ShooterConstants::TURRET_UNLOAD_AIMED); //TODO get value

    return error;
}

double Turret::calcPID()
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    yawDT_ = time - yawPrevTime_;

    prevTime_ = time;

    deltaYaw_ = yaw_ - prevYaw_;
    //cout << "yaws: " << yaw_ << ", " << prevYaw_ << ", " << deltaYaw_ << "," << yawVel_ << endl;

    if(deltaYaw_ != 0)
    {
        prevYaw_ = yaw_;

        if(abs(deltaYaw_) > 300)
        {
            deltaYaw_ = (deltaYaw_ > 0) ? deltaYaw_ - 360 : deltaYaw_ + 360;
        }

        yawVel_ = deltaYaw_ / yawDT_;
        yawPrevTime_ = time;
    }

    double error = calcError();
    
    double deltaError = (error - prevError_) / dT_;
    integralError_ += error * dT_;

    if(abs(prevError_) < 2.5 && abs(error > 5)) //TODO get value, probably same as above
    {
        deltaError = 0;
        integralError_ = 0;
    } 
    prevError_ = error;

    double power = (tkP_*error) + (tkI_*integralError_) + (tkD_*deltaError);

    if(state_ == CLIMB)
    {
        return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3);   
    }

    power = std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3);
    //power = 0;
    power += calcAngularFF();
    power += calcLinearFF();
    //frc::SmartDashboard::PutNumber("LTFF", calcLinearFF());

    //TODO, disable voltage limit for ffs... like what?

    return power;
    //return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3); //TODO get cap value
}

void Turret::setInPos(double pos)
{
    inPos_ = pos;
    Helpers::normalizeAngle(inPos_);
}

//1, 24
//2, 64
//3, 77.419
//4, 80.898876