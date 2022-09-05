#include "Shooter.h"

Shooter::Shooter(Limelight* limelight, SwerveDrive* swerveDrive, Channel* channel) : limelight_(limelight), swerveDrive_(swerveDrive), channel_(channel), flywheelMaster_(ShooterConstants::FLYWHEEL_MASTER_ID), flywheelSlave_(ShooterConstants::FLYWHEEL_SLAVE_ID), kickerMotor_(ShooterConstants::KICKER_ID), flyTrajectoryCalc_(maxV, maxA, kP, kD, kV, kA, kVI), turret_(limelight_, swerveDrive_)
{
    flywheelMaster_.SetInverted(TalonFXInvertType::Clockwise);
    flywheelSlave_.Follow(flywheelMaster_);
    flywheelSlave_.SetInverted(InvertType::OpposeMaster);

    rangeAdjustment_ = 0;
    state_ = IDLE;

    unloadStarted_ = false;
    unloadShooting_ = false;
    shootStarted_ = false;
    shooting_ = false;

    createMap(ShooterConstants::SHOTS_FILE_NAME, shotsMap_);
    //createMap(ShooterConstants::LOW_ANGLE_SHOTS_FILE_NAME, lowAngleShotsMap_);
    hasMaps_ = true;
}

void Shooter::createMap(string fileName, map<double, tuple<double, double, double>> &map)
{
    if(hasMaps_)
    {
        return;
    }
    ifstream infile(fileName);
    
    cout << "started creating a map" << endl;

    string data;
    double distance, angle, velocity, partDer;

    bool valid;
    std::size_t c1, c2, c3;

    while(getline(infile, data))
    {
        valid = true;

        c1 = data.find(", ");
        if(c1 != string::npos)
        {
            distance = stod(data.substr(0, c1));

            c2 = data.find(", ", c1 + 1);
            if(c2 != string::npos)
            {
                angle = stod(data.substr(c1 + 2, c2));

                c3 = data.find(", ", c2 + 1);
                if(c3 != string::npos)
                {
                    velocity = stod(data.substr(c2 + 2, c3));
                    partDer = stod(data.substr(c3 + 2));
                }
                else
                {
                    valid = false;
                }
                
            }
            else
            {
                valid = false;
            }
        }
        else
        {
            valid = false;
        }

        if(valid)
        {
            tuple<double, double, double> shotData(angle, velocity, partDer);
            pair<double, tuple<double, double, double>> distancePoint(distance, shotData);

            shotsMap_.insert(distancePoint);
            ++mapPoints_;
            //cout << "added a point" << endl;
        }
    }

    //hasMap_ = true;
    //frc::SmartDashboard::PutNumber("Map Points", mapPoints_);
    infile.close();
}

Shooter::State Shooter::getState()
{
    return state_;
}

void Shooter::setState(State state)
{
    state_ = state;
}

// void Shooter::setPID(double p, double i, double d)
// {
//     fKp_ = p;
//     fKi_ = i;
//     fKd_ = d;
// }

// void Shooter::setHoodPID(double p, double i, double d)
// {
//     hood_.setPID(p, i, d);
// }

void Shooter::dewindIntegral()
{
    integralError_ = 0;
}

void Shooter::increaseRange()
{
    rangeAdjustment_ += ShooterConstants::Kr;
}

void Shooter::decreaseRange()
{
    rangeAdjustment_ -= ShooterConstants::Kr;
}

void Shooter::setTurretManualVolts(double manualVolts)
{
    turret_.setManualVolts(manualVolts);
}

void Shooter::clearBallShooting()
{
    shootStarted_ = false;
    shooting_ = false;
}

void Shooter::periodic(double yaw)
{
    if(state_ == UNLOADING)
    {
        limelight_->lightOn(false);
    }
    else
    {
        unloadStarted_ = false;
        unloadShooting_ = false;
    }

    yaw_ = yaw;
    swerveDrive_->calcOdometry(turret_.getAngle(), false);

    //frc::SmartDashboard::PutBoolean("map", hasMap_);

    double hoodAngle, velocity, turretOffset, partDer, distance;
    //frc::SmartDashboard::PutBoolean("found target", swerveDrive->foundGoal());

    /*if(prevDistance_ == -1 || !limelight_->hasTarget() || abs(swerveDrive_->getGoalXVel()) > 0.01 || abs(swerveDrive_->getGoalYVel()) > 0.01 || !turret_.isAimed())
    {
        distance = limelight_->calcDistance();
        //distance = swerveDrive_->getDistance(turret_.getAngle());
        prevDistance_ = distance;
    }
    else
    {
        distance = prevDistance_;
    }*/
    distance = swerveDrive_->getDistance(turret_.getAngle());

    double swerveDistance = swerveDrive_->getDistance(turret_.getAngle());
    frc::SmartDashboard::PutNumber("SDistance", swerveDistance);
    frc::SmartDashboard::PutNumber("FDistance", distance);
    frc::SmartDashboard::PutNumber("LDistance", limelight_->calcDistance());

    //distance = 5;

    //distance = frc::SmartDashboard::GetNumber("InDist", -1); //Comment out below stuff if using

    if(distance != -1)
    {
        distance += (rangeAdjustment_ + LimelightConstants::LIMELIGHT_TO_BALL_CENTER_DIST) - 0.1524; //TODO, change or something
    }
    else if(swerveDrive_->foundGoal())
    {
        hasShot_ = false;
        distance = swerveDrive_->getDistance(turret_.getAngle());
    }
    else
    {
        distance = 0;
        hasShot_ = false;
    }
    
    frc::SmartDashboard::PutNumber("Range Adjustment", rangeAdjustment_);
    auto shot = shotsMap_.upper_bound(distance);
    if (shot != shotsMap_.begin() && shot != shotsMap_.end()) //TOOD test with distance?
    {
        //TODO disable interpolation when not using Andrew's points
        /*double higher = shot->first;
        double highHood = get<0>(shot->second);
        double highVel = get<1>(shot->second);*/

        --shot;
        partDer = get<2>(shot->second);

        //swerveDrive_->getGoalXVel();
        //swerveDrive_->getGoalYVel();
        tuple<double, double, double> shotVals = calcShootingWhileMoving(get<0>(shot->second), get<1>(shot->second), swerveDrive_->getGoalXVel(), swerveDrive_->getGoalYVel());
        hoodAngle = get<0>(shotVals);
        velocity = get<1>(shotVals);
        turretOffset = get<2>(shotVals);

        /*if(swerveDrive_->getGoalYVel() > 0.4 && state_ == REVING)
        {
            auto lowAngleShot = lowAngleShotsMap_.upper_bound(distance);
            if(lowAngleShot != lowAngleShotsMap_.begin() && lowAngleShot != lowAngleShotsMap_.end())
            {
                --lowAngleShot;
                partDer = get<2>(lowAngleShot->second);
                shotVals = calcShootingWhileMoving(get<0>(lowAngleShot->second), get<1>(lowAngleShot->second), swerveDrive_->getGoalXVel(), swerveDrive_->getGoalYVel());
                hoodAngle = get<0>(shotVals);
                velocity = get<1>(shotVals);
                turretOffset = get<2>(shotVals);
            }
        }*/

        /*hoodAngle = get<0>(shot->second);
        velocity = get<1>(shot->second);
        turretOffset = 0;*/

        //double skew = (distance - shot->first) / (higher - shot->first);

        //hoodAngle += skew * (highHood - hoodAngle);
        //velocity += skew * (highVel - velocity);

        hasShot_ = true;

        frc::SmartDashboard::PutNumber("MAng", hoodAngle);
        frc::SmartDashboard::PutNumber("MVel", velocity);
        frc::SmartDashboard::PutNumber("MTOff", turretOffset);
    }
    else
    {
        velocity = 0;
        hoodAngle = ShooterConstants::MAX_HOOD_ANGLE;
        //hoodAngle = (ShooterConstants::MIN_HOOD_ANGLE + ShooterConstants::MAX_HOOD_ANGLE) / 2;
        turretOffset = 0;
        partDer = 1;
        hasShot_ = false;
    }

    //hoodAngle = setHoodTicks_;
    //frc::SmartDashboard::PutNumber("V", flywheelMaster_.GetSelectedSensorVelocity() * 20 * M_PI * ShooterConstants::FLYWHEEL_RADIUS / (GeneralConstants::TICKS_PER_ROTATION * ShooterConstants::FLYWHEEL_GEAR_RATIO));

    if(partDer > 10.5) //Change back to 0.5, maybe lower to 0.3
    {
        hasShot_ = false; //TODO set value, see how auto shoot works?
    }

    if(hoodAngle > ShooterConstants::MAX_HOOD_ANGLE || hoodAngle < ShooterConstants::MIN_HOOD_ANGLE/*ShooterConstants::MAX_HOOD_TICKS*/)
    {
        hoodAngle = ShooterConstants::MAX_HOOD_ANGLE;
        hasShot_ = false;
    }

    if(velocity < 0 || velocity > 30)
    {
        velocity = 0;
        hasShot_ = false;
    }

    if(abs(turretOffset) > 90)
    {
        std::cout << "Bro wtf is even happening with your math" << std::endl;
        turretOffset = 0;
        hasShot_ = false;
    }

    frc::SmartDashboard::PutBoolean("Hood Ready", hood_.isReady());
    frc::SmartDashboard::PutBoolean("Flywheel Ready", flywheelReady_);
    frc::SmartDashboard::PutBoolean("Turret Ready", turret_.isAimed());
    frc::SmartDashboard::PutBoolean("Has Shot", hasShot_);

    //hasShot_ = true;
    shotReady_ = (flywheelReady_ && hood_.isReady() && turret_.isAimed() && hasShot_); //TODO something with have ball?

    //frc::SmartDashboard::PutBoolean("badIdea", channel_.badIdea());
    
    //double kickerVel = frc::SmartDashboard::GetNumber("K", ShooterConstants::KICKER_VOLTS);
    //velocity = frc::SmartDashboard::GetNumber("InV", 0);
    //velocity = std::clamp(velocity, 0.0, ShooterConstants::MAX_VELOCITY);
    //hoodAngle = frc::SmartDashboard::GetNumber("InA", 0);
    //hoodAngle = std::clamp(hoodAngle, (double)ShooterConstants::MAX_HOOD_TICKS, 0.0);
    
    //double hoodV = frc::SmartDashboard::GetNumber("InHV", 0);
    //hood_.setInVolts(hoodV);
    //turretOffset = 0;
    frc::SmartDashboard::PutBoolean("Unloading", (state_ == UNLOADING));

    if(channel_->getBallCount() > 0 && shootStarted_)
    {
        state_ = SHOOTING;
    }

    switch(state_)
    {
        case OUTAKING:
        {
            hood_.setWantedPos(hoodAngle);
            hood_.setState(Hood::AIMING);
            
            turret_.setState(Turret::TRACKING);
            //turret_.setState(Turret::MANUAL);

            flywheelMaster_.SetVoltage(units::volt_t (0));
            kickerMotor_.SetVoltage(units::volt_t(-6));
            dewindIntegral();

            break;
        }
        case IDLE:
        {
            hood_.setState(Hood::IDLE);
            turret_.setState(Turret::IDLE);
            dewindIntegral();
            break;
        }
        case TRACKING:
        {
            hood_.setWantedPos(hoodAngle);
            //hood_.setWantedPos(0); //-100
            hood_.setState(Hood::AIMING);
            

            turret_.setState(Turret::TRACKING);
            //turret_.setState(Turret::MANUAL);

            kickerMotor_.SetVoltage(units::volt_t(0));

            flywheelMaster_.SetVoltage(units::volt_t (0));
            //units::volt_t volts {calcFlyPID(velocity)};
            //flywheelMaster_.SetVoltage(volts);
            dewindIntegral();

            //unloadStarted_ = false;
            //unloadShooting_ = false;
            shootStarted_ = false;
            shooting_ = false;
            break;
        }
        case REVING:
        {
            hood_.setWantedPos(hoodAngle);
            //hood_.setWantedPos(0); //-100
            hood_.setState(Hood::AIMING);
            
            turret_.setState(Turret::TRACKING);
            //turret_.setState(Turret::MANUAL);

            kickerMotor_.SetVoltage(units::volt_t(0));

            units::volt_t volts {calcFlyPID(velocity)};
            //units::volt_t volts{calcFlyVolts(velocity)};
            flywheelMaster_.SetVoltage(volts);

            shootStarted_ = false;
            shooting_ = false;
            break;
        }
        case SHOOTING: //TODO combine for auto shoot later, hood anti-windup
        {
            hood_.setWantedPos(hoodAngle);
            //hood_.setWantedPos(0); //-100
            hood_.setState(Hood::AIMING);

            turret_.setState(Turret::TRACKING);
            //turret_.setState(Turret::MANUAL);

            //cout << velocity << endl;

            units::volt_t volts {calcFlyPID(velocity)};
            //units::volt_t volts{calcFlyVolts(velocity)};
            //units::volt_t volts {calcFlyPID(vel_)};
            //units::volt_t volts {calcFlyPID(7000)};

            //units::volt_t volts{frc::SmartDashboard::GetNumber("FINV", 0)};

            flywheelMaster_.SetVoltage(volts);
            
            //frc::SmartDashboard::PutNumber("FV", flywheelMaster_.GetSelectedSensorVelocity());
            //1, 580
            //2, 2310
            //3, 4090
            //4, 5950
            //5, 7800, can go above 8000
            //6, 9700, can go 
            //7, 11450
            //8, 12960
            //9, 14250
            //10, 15550
            //11, 16550
            //12, 16675

            //4000, 
            //5000, 5.1098823529352
            //6000, 6.4346666666582
            //7000, 7.393021276589799
            //8000, 8.474926829258999
            //9000, 9.652
            //10000, 10.529454545443
            //11000, 
            //12000, 11.5824
            //13000, 12.4097142857034

            //flywheelMaster_.SetVoltage(units::volt_t(6));

            if(shotReady_)
            {
                //frc::SmartDashboard::PutBoolean("Shooting", true);
                shootStarted_ = true;
                kickerMotor_.SetVoltage(units::volt_t(ShooterConstants::KICKER_VOLTS)); //TODO tune value
            }
            else
            {
                //frc::SmartDashboard::PutBoolean("Shooting", false);
                kickerMotor_.SetVoltage(units::volt_t(0));
            }

            if(shootStarted_ && flywheelMaster_.GetSelectedSensorVelocity() < (wantedSensVel_ - 1000)/*flywheelMaster_.GetSupplyCurrent() > ShooterConstants::UNLOADING_CURRENT*/)
            {
                shooting_ = true;
            }

            if(shooting_ && flywheelMaster_.GetSelectedSensorVelocity() > (wantedSensVel_ - 500)/*flywheelMaster_.GetSupplyCurrent() < ShooterConstants::UNLOADING_CURRENT_LOW*/)
            {
                shootStarted_ = false;
                shooting_ = false;
                channel_->decreaseBallCount();
            }
            break;
        }
        case UNLOADING:
        {
            //hood_.setWantedPos(-2000);
            hood_.setWantedPos(50);
            hood_.setState(Hood::AIMING);

            //frc::SmartDashboard::PutNumber("FVEL", flywheelMaster_.GetSelectedSensorVelocity());
            //frc::SmartDashboard::PutNumber("FCUR", flywheelMaster_.GetSupplyCurrent());
            turret_.setState(Turret::UNLOADING);
            //turret_.setState(Turret::MANUAL);

            //units::volt_t volts {calcFlyPID(7000)}; //TODO get value or make a distance map
            units::volt_t volts {calcFlyPID(7.6032)};
            flywheelMaster_.SetVoltage(volts);

            if(turret_.unloadReady() && flywheelEjectReady_)
            {
                kickerMotor_.SetVoltage(units::volt_t(ShooterConstants::KICKER_VOLTS + 3));
                unloadStarted_ = true;
            }
            else
            {
                kickerMotor_.SetVoltage(units::volt_t(0));
            }

            if(unloadStarted_ && flywheelMaster_.GetSelectedSensorVelocity() < 6000/*flywheelMaster_.GetSupplyCurrent() > ShooterConstants::UNLOADING_CURRENT*/)
            {
                unloadShooting_ = true;
            }

            if(unloadShooting_ && flywheelMaster_.GetSelectedSensorVelocity() > 6500/*flywheelMaster_.GetSupplyCurrent() < ShooterConstants::UNLOADING_CURRENT_LOW*/)
            {
                state_ = TRACKING;
                unloadStarted_ = false;
                unloadShooting_ = false;
                channel_->decreaseBallCount();
            }

            break;
        }
        case MANUAL:
        {
            turret_.setState(Turret::MANUAL);
            hood_.setState(Hood::IDLE);

            flywheelMaster_.SetVoltage(units::volt_t(0));
            dewindIntegral();
            break;
        }
        case CLIMB:
        {
            turret_.setState(Turret::CLIMB);
            hood_.setState(Hood::IDLE);

            flywheelMaster_.SetVoltage(units::volt_t(0));
            dewindIntegral();
            break;
        }
    }

    hood_.periodic();
    turret_.periodic(yaw_, turretOffset);
    channel_->periodic();
}

void Shooter::reset()
{
    hood_.reset();
    turret_.reset();
    dewindIntegral();
    rangeAdjustment_ = 0;
}

void Shooter::zeroHood()
{
    hood_.setState(Hood::ZEROING);
}

double Shooter::linVelToSensVel(double velocity)
{
    //a = 66.0934, b = -73.0616, c = 3734.77
    
    wantedSensVel_ = (66.0934 * velocity * velocity) - (73.0616 * velocity) + 3734.77;
    return wantedSensVel_;

    //return ShooterConstants::FLYWHEEL_GEAR_RATIO * (velocity / ShooterConstants::FLYWHEEL_RADIUS) * GeneralConstants::TICKS_PER_ROTATION / ( 20 * M_PI);
}

double Shooter::calcFlyPID(double velocity)
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    double setAngVel = linVelToSensVel(velocity);
    double error = setAngVel - flywheelMaster_.GetSelectedSensorVelocity();

    //double error = velocity - flywheelMaster_.GetSelectedSensorVelocity();

    //frc::SmartDashboard::PutNumber("setFV", setAngVel);
    frc::SmartDashboard::PutNumber("Ferror", error);
    //frc::SmartDashboard::PutNumber("FV", flywheelMaster_.GetSelectedSensorVelocity());

    integralError_ += error * dT_;
    double deltaError = (error - prevError_) / dT_;

    flywheelReady_ = (abs(error) < ShooterConstants::FLYWHEEL_READY/* && deltaError > -300 && abs(deltaError) < 500*/); //TODO get value
    flywheelEjectReady_ = (abs(error) < ShooterConstants::FLYWHEEL_EJECT_READY);
    if(abs(prevError_) < 40 && error > 100) //TODO get value, probably same as above
    {
        deltaError = 0;
        //integralError_ = 0;
    }
    prevError_ = error;
    prevVelocity_ = flywheelMaster_.GetSelectedSensorVelocity();

    double feedForward = (abs(setAngVel) - ShooterConstants::FLYWHEEL_FF_INTERCEPT) / ShooterConstants::FLYWHEEL_FF;
    if(setAngVel == 0)
    {
        feedForward = 0;
    }
    else if(setAngVel < 0)
    {
        setAngVel *= -1;
    }

    double power = (fKp_ * error) + (fKi_ * integralError_) + (fKd_ * deltaError) + feedForward;

    if(error > 6000) //TODO get values
    {
        power += 2;
    }

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

double Shooter::calcFlyVolts(double velocity)
{
    double volts;
    double setAngVel = linVelToSensVel(velocity);
    double error = setAngVel - flywheelMaster_.GetSelectedSensorVelocity();

    flywheelReady_ = (abs(error) < ShooterConstants::FLYWHEEL_READY);
    flywheelEjectReady_ = (abs(error) < ShooterConstants::FLYWHEEL_EJECT_READY);
    //frc::SmartDashboard::PutNumber("FError", error);

    if(abs(setAngVel - setTrajectoryVel_) > 100 && initTrajectory_) //TODO get value
    {
        setTrajectoryVel_ = setAngVel;
        //double vel = flyTrajectoryCalc_.getVelProfile().second;
        double vel = flywheelMaster_.GetSelectedSensorVelocity() * 10;
        flyTrajectoryCalc_.generateVelTrajectory(setTrajectoryVel_, vel);
    }
    if(!initTrajectory_)
    {
        initTrajectory_ = true;
        setTrajectoryVel_ = setAngVel;
        double vel = flywheelMaster_.GetSelectedSensorVelocity() * 10;
        flyTrajectoryCalc_.generateVelTrajectory(setAngVel, vel);
    }

    if(initTrajectory_)
    {
        double vel = flywheelMaster_.GetSelectedSensorVelocity() * 10;
        volts = flyTrajectoryCalc_.calcVelPower(vel);
    }
    else
    {
        volts = 0;
    }

    //frc::SmartDashboard::PutNumber("FVEL", flywheelMaster_.GetSelectedSensorVelocity());
    //frc::SmartDashboard::PutNumber("FVOLTS", volts);
    //frc::SmartDashboard::PutNumber("WVEL", flyTrajectoryCalc_.getVelProfile().second);

    return volts;
}

tuple<double, double, double> Shooter::calcShootingWhileMoving(double hoodAngle, double velocity, double goalXVel, double goalYVel)
{
    double zVel = velocity * sin(hoodAngle * M_PI / 180);
    double yVel = velocity * cos(hoodAngle * M_PI / 180) - goalYVel;
    double xVel = -goalXVel;

    double xyVel = sqrt(xVel * xVel + yVel * yVel);
    double newVel = sqrt(xyVel * xyVel + zVel * zVel);

    double turretAngle, newHoodAngle;
    if(yVel == 0 && xVel == 0)
    {
        turretAngle = 0;
    }
    else
    {
        turretAngle = -(atan2(yVel, xVel) * 180 / M_PI) + 90;
    }
    if(zVel == 0 && xyVel == 0)
    {
        newHoodAngle = ShooterConstants::MAX_HOOD_ANGLE;
        hasShot_ = false;
    }
    else
    {
        newHoodAngle = atan2(zVel, xyVel) * 180 / M_PI;
    }

    return tuple<double, double, double> (newHoodAngle, newVel, turretAngle);

}