#include "Turret.h"

Turret::Turret(Limelight *limelight, SwerveDrive *swerveDrive) : turretMotor_(ShooterConstants::TURRET_ID), trajectoryCalc_(maxV, maxA, kP, kD, kV, kA, kVI)
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

    switch (state_) // TODO clean up/combine cases after the switch
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
        swerveDrive_->setFoundGoal(false);
        // frc::SmartDashboard::PutNumber("TX", limelight_->getAdjustedX());
        //  frc::SmartDashboard::PutNumber("TV", turretMotor_.GetSelectedSensorVelocity());
        //  calcError(); //for printing values
        limelight_->lightOn(true);
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
    // return true;
    return aimed_;
}

bool Turret::unloadReady()
{
    // return true;
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
    if (!limelight_->hasTarget() && !swerveDrive_->foundGoal())
    {
        turretMotor_.SetVoltage(units::volt_t(0));
        limelight_->lightOn(true);
    }
    else
    {
        double time = timer_.GetFPGATimestamp().value();
        dT_ = time - prevTime_;
        yawDT_ = time - yawPrevTime_;

        prevTime_ = time;

        deltaYaw_ = yaw_ - prevYaw_;

        if (deltaYaw_ != 0)
        {
            prevYaw_ = yaw_;

            if (abs(deltaYaw_) > 300)
            {
                deltaYaw_ = (deltaYaw_ > 0) ? deltaYaw_ - 360 : deltaYaw_ + 360;
            }

            yawVel_ = deltaYaw_ / yawDT_;
            yawPrevTime_ = time;
        }

        double volts = calcPID();
        // double volts = calcProfileVolts();

        if (volts > 0 && getAngle() > 175)
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
            //turretMotor_.SetVoltage(units::volt_t(volts));
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
    if (xDist != 0 || yDist != 0)
    {
        angToHangar = -(atan2(yDist, xDist) * 180 / M_PI) - 90;
    }

    double angToGoal = 0;
    if (x != 0 || y != 0)
    {
        angToGoal = -(atan2(y, x) * 180 / M_PI) - 90;
    }

    angToHangar += 360 * 10;
    angToHangar = ((int)floor(angToHangar) % 360) + (angToHangar - floor(angToHangar));

    angToGoal += 360 * 10;
    angToGoal = ((int)floor(angToGoal) % 360) + (angToGoal - floor(angToGoal));

    if (abs(angToHangar - angToGoal) < 10) // TODO get value
    {
        angToHangar += (angToHangar > angToGoal) ? 10 : -10; // TODO incorporate goal diameter?
    }

    unloadAngle_ = 180 + angToHangar - yaw_;
    Helpers::normalizeAngle(unloadAngle_);
}

double Turret::calcAngularFF()
{
    //double rff = -yawVel_ / 82;

    if (abs(yawVel_) < 5)
    {
        return 0;
    }

    double ff = -(abs(yawVel_) - ShooterConstants::TURRET_FF_INTERCEPT) / ShooterConstants::TURRET_FF;
    if (yawVel_ < 0)
    {
        ff *= -1;
    }

    return ff;
}

double Turret::calcLinearFF()
{
    double vel = swerveDrive_->getGoalXVel();
    double x = swerveDrive_->getX();
    double y = swerveDrive_->getY();
    double distance = sqrt(x * x + y * y);

    double degPerSec = (-vel / distance) * 180 / M_PI;
    if (abs(degPerSec) < 5)
    {
        return 0;
    }

    double ff = (abs(degPerSec) - ShooterConstants::TURRET_FF_INTERCEPT) / ShooterConstants::TURRET_FF;
    if (degPerSec < 0)
    {
        ff *= -1;
    }

    return ff * 0.5;
}

double Turret::calcError()
{
    if (!swerveDrive_->foundGoal())
    {
        return 0;
    }
    double error, goalError;
    if (state_ == UNLOADING)
    {
        error = unloadAngle_ - getAngle();
    }
    else if (state_ == CLIMB)
    {
        error = -getAngle();
        return error;
    }
    else if (limelight_->hasTarget())
    {
        error = offset_ + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET;
    }
    else
    {
        double wantedTurretAng = (180 - swerveDrive_->getRobotGoalAng()) + offset_;
        Helpers::normalizeAngle(wantedTurretAng);

        error = wantedTurretAng - getAngle();
    }

    if (limelight_->hasTarget())
    {
        goalError = limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET;
    }
    else
    {
        double wantedGoalAng = (180 - swerveDrive_->getRobotGoalAng());
        Helpers::normalizeAngle(wantedGoalAng);

        goalError = wantedGoalAng - getAngle();
    }

    frc::SmartDashboard::PutNumber("Terror", error);

    if (abs(error + getAngle()) > 180)
    {
        if (getAngle() > 0 && error > 0)
        {
            error -= 360;
        }

        else if (getAngle() < 0 && error < 0)
        {
            error += 0;
        }

        // error = (error > 0) ? error - 360 : error + 360;
    }

    if (abs(goalError) > 40 && !limelight_->hasTarget()) // COMP disable probably
    {
        limelight_->lightOn(false);
    }
    else if (state_ != CLIMB && state_ != UNLOADING)
    {
        limelight_->lightOn(true);
    }

    aimed_ = (abs(error) < ShooterConstants::TURRET_AIMED);              // TODO get value, change back to 2.5
    unloadReady_ = (abs(error) < ShooterConstants::TURRET_UNLOAD_AIMED); // TODO get value

    return error;
}

double Turret::calcPID()
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    yawDT_ = time - yawPrevTime_;

    prevTime_ = time;

    deltaYaw_ = yaw_ - prevYaw_;

    if (deltaYaw_ != 0)
    {
        prevYaw_ = yaw_;

        if (abs(deltaYaw_) > 300)
        {
            deltaYaw_ = (deltaYaw_ > 0) ? deltaYaw_ - 360 : deltaYaw_ + 360;
        }

        yawVel_ = deltaYaw_ / yawDT_;
        yawPrevTime_ = time;
    }

    double error = calcError();

    double deltaError = (error - prevError_) / dT_;
    integralError_ += error * dT_;

    if (abs(prevError_) < 2.5 && abs(error > 5)) // TODO get value, probably same as above
    {
        deltaError = 0;
        integralError_ = 0;
    }
    prevError_ = error;

    double power = (tkP_ * error) + (tkI_ * integralError_) + (tkD_ * deltaError);

    if (state_ == CLIMB)
    {
        return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3);
    }

    power = std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3);
    // power = 0;
    power += calcAngularFF();
    power += calcLinearFF();
    // frc::SmartDashboard::PutNumber("LTFF", calcLinearFF());

    // return power;
    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE); // TODO get cap value
}

void Turret::setInPos(double pos)
{
    inPos_ = pos;
    Helpers::normalizeAngle(inPos_);
}

double Turret::calcProfileVolts()
{
    double volts;
    double error = calcError();

    double pos = getAngle();
    frc::SmartDashboard::PutNumber("TP", pos);

    double vel = turretMotor_.GetSelectedSensorVelocity() * 10 / ShooterConstants::TICKS_PER_TURRET_DEGREE;
    frc::SmartDashboard::PutNumber("TV", vel);

    double setPos = pos + error;
    frc::SmartDashboard::PutNumber("TSP", setPos);
    if (abs(setPos > 180))
    {
        frc::SmartDashboard::PutNumber("SPTHING DIED", setPos);
        frc::SmartDashboard::PutNumber("ETHING DIED", error);
        frc::SmartDashboard::PutBoolean("THING DIED", true);
        return 0;
    }

    // double inPos = frc::SmartDashboard::GetNumber("InT", getAngle());
    // Helpers::normalizeAngle(inPos);
    // error = inPos - getAngle();
    // setPos = inPos;

    // error = inPos_ - getAngle();
    // setPos = inPos_;

    if (initTrajectory_ && abs(setPos - currentSetPos_) < 3)
    {
        double wantedAcc = get<0>(trajectoryCalc_.getProfile());
        double wantedVel = get<1>(trajectoryCalc_.getProfile());
        if (abs(error) > ShooterConstants::TURRET_AIMED && wantedVel == 0 && wantedAcc == 0 && abs(vel) < 1)
        {
            trajectoryCalc_.generateTrajectory(pos, setPos, vel);
        }
    }
    else
    {
        initTrajectory_ = true;
        currentSetPos_ = setPos;

        trajectoryCalc_.generateTrajectory(pos, setPos, vel);
    }

    // if(vel != 0 || !initTrajectory_)
    // {
    //     trajectoryCalc_.generateTrajectory(pos, setPos, vel);
    //     initTrajectory_ = true;
    // }
    // else if (get<1>(trajectoryCalc_.getProfile()) == 0)
    // {
    //     trajectoryCalc_.generateTrajectory(pos, setPos, vel);
    // }

    volts = trajectoryCalc_.calcPower(pos, vel);

    if (state_ != CLIMB && abs(getAngle()) < 178)
    {
        // volts += calcAngularFF();
        // volts += calcLinearFF();
    }

    return volts;
}

// 1, 24
// 2, 64
// 3, 77.419
// 4, 80.898876