#include "SwerveModule.h"

SwerveModule::SwerveModule(int turnID, int driveID, int cancoderID, double offset) : turnMotor_(turnID, "Drivebase"), driveMotor_(driveID, "Drivebase"), cancoder_(cancoderID, "Drivebase"), trajectoryCalc_(maxV, maxA, kP, kD, kV, kA, kVI), offset_(offset)
{
    turnMotor_.SetInverted(TalonFXInvertType::CounterClockwise);
    driveMotor_.SetInverted(TalonFXInvertType::Clockwise);

    turnMotor_.SetNeutralMode(NeutralMode::Brake);
    driveMotor_.SetNeutralMode(NeutralMode::Brake);

    id_ = std::to_string(driveID);

    initTrajectory_ = false;
    cancoder_.ClearStickyFaults();//TODO check what this actually does
}

void SwerveModule::periodic(double driveSpeed, double angle)
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;
    
    move(driveSpeed, angle);
}

void SwerveModule::move(double driveSpeed, double angle)
{
    //frc::SmartDashboard::PutNumber(id_ + " Wanted speed", driveSpeed);
    //frc::SmartDashboard::PutNumber(id_ + " Wanted angle", angle);

    /*if(abs(setTrajectoryPos_ - angle) > 0.1 && initTrajectory_) //TODO get value
    {
        setTrajectoryPos_ = angle;

        //double pos = get<2>(trajectoryCalc_.getProfile()) + posOffset_;
        //posOffset_ = pos;
        double pos = getAngle();
        posOffset_ = pos;

        double error = findError(angle, pos);

        //double vel = get<1>(trajectoryCalc_.getProfile());
        double vel = cancoder_.GetVelocity();

        trajectoryCalc_.generateTrajectory(0, error, vel);
    }
    else if(!initTrajectory_)
    {
        initTrajectory_ = true;
        setTrajectoryPos_ = angle;
        posOffset_ = getAngle();

        double error = findError(angle, getAngle());

        trajectoryCalc_.generateTrajectory(0, error, cancoder_.GetVelocity());
    }
    
    if(initTrajectory_)
    {
        double pos = getAngle() - posOffset_;
        Helpers::normalizeAngle(pos);
        double vel = cancoder_.GetVelocity();

        units::volt_t swivelVols(trajectoryCalc_.calcPower(pos, vel));
        turnMotor_.SetVoltage(swivelVols);
    }*/

    //turnMotor_.SetVoltage(units::volt_t(4));
    //1, 3.77, 95.49, 95.49
    //2, 0.91, 395.60, 197.8
    //3, 0.55, 654.54, 218.18
    //4, 0.4, 900, 225
    //frc::SmartDashboard::PutNumber(id_ + " pos", cancoder_.GetAbsolutePosition());

    units::volt_t turnVolts{calcAngPID(angle)};
    turnMotor_.SetVoltage(turnVolts);
    //turnMotor_.SetVoltage(units::volt_t(frc::SmartDashboard::GetNumber("smiv", 0)));
    //frc::SmartDashboard::PutNumber(id_ + " Turn volts", turnVolts.value());

    units::volt_t driveVolts{direction_ * calcDrivePID(driveSpeed)};
    driveMotor_.SetVoltage(driveVolts);
    //frc::SmartDashboard::PutNumber(id_ + " Drive volts", driveVolts.value());

    frc::SmartDashboard::PutNumber(id_ + " VEL", cancoder_.GetVelocity());
    
}

//1, 107
//2, 400
//3, 666
//4, 921
//5, 1180
//6, 1440p
//7, 1700
//8, 1960
//9, 2230
//10, 2500
//11, 2730
//12, 2855

double SwerveModule::calcAngPID(double setAngle)
{

    double error = findError(setAngle, getAngle());
    //frc::SmartDashboard::PutNumber(id_ + "Angle error", error);

    aIntegralError_ += error * dT_;
    double deltaError = (error - aPrevError_) / dT_;
    if(abs(aPrevError_) < 2 && error > 5) //TODO get value
    {
        deltaError = 0;
        aIntegralError_ = 0;
    } 
    aPrevError_ = error;

    double power = (akP_*error) + (akI_*aIntegralError_) + (akD_*deltaError); //TODO implement integral anti-windup or just don't use kI

    //return 1.5;
    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

/*void SwerveModule::normalizeAngle() //test later for math mistake
{
    //angle_ = turnMotor_.GetSelectedSensorPosition() * GEAR_RATIO * 360.0 / 2048.0;
    angle_ = cancoder_.GetAbsolutePosition();

    //angle_ += (360 * 100);

    //double angleDec = angle_ - floor(angle_);

    //angle_ = ((int)floor(angle_) % 360);
    //angle_ += angleDec;

    angle_ -= 360 * floor(angle_ / 360 + 0.5); 

}*/

/*double rpmToTicksPerDS(double rpm)
{
        return (rpm * TICKS_PER_ROTATION) / (600);
}*/

double SwerveModule::calcDrivePID(double driveSpeed)
{
    double velocity = (driveSpeed * GeneralConstants::MAX_RPM * GeneralConstants::TICKS_PER_ROTATION) / 600;
    double error = velocity - driveMotor_.GetSelectedSensorVelocity();

    //frc::SmartDashboard::PutNumber(id_ + "velocity error", error);

    dIntegralError_ += error * dT_;
    double deltaError = (error - dPrevError_) / dT_;
    dPrevError_ = error;

    if(abs(deltaError) < 40 && error > 100) //TODO get value, also test if needed
    {
        deltaError = 0;
        dIntegralError_ = 0;
    }

    //double radPSec = (driveSpeed * GeneralConstants::MAX_RPM) * 2 * M_PI / 60;
    //double feedForward = radPSec / GeneralConstants::Kv;
    double feedForward = GeneralConstants::MAX_VOLTAGE * driveSpeed;
    //feedForward = 0;

    double power = (dkP_*error) + (dkI_*aIntegralError_) + (dkD_*deltaError) + feedForward;

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);

}

double SwerveModule::findError(double setAngle, double angle)
{
    double rawError = setAngle - angle;

    //frc::SmartDashboard::PutNumber(id_ + "Angle", getAngle());

    Helpers::normalizeAngle(rawError);

    if(abs(rawError) > 90)
    {
        direction_ = -1;
    }
    else
    {
        direction_ = 1;
    }
    
    return (abs(rawError) <= 90) ? rawError : (rawError > 0) ? rawError - 180 : rawError + 180;
    /*if(abs(error) <= 90)
    {
        return error;
    }
    else
    {
        return (error > 0) ? error - 180 : error + 180;
    }*/

    //return rawError;

    

    /*double error = setAngle - getAngle();
    frc::SmartDashboard::PutNumber(id_ + "Angle", getAngle());*/

    //return error;
}

double SwerveModule::getDriveVelocity()
{
    return (driveMotor_.GetSelectedSensorVelocity() / 2048) * 10 * SwerveConstants::DRIVE_GEAR_RATIO * 2 * M_PI * SwerveConstants::TREAD_RADIUS;
}

double SwerveModule::getAngle()
{
    double angle = cancoder_.GetAbsolutePosition() + offset_;
    Helpers::normalizeAngle(angle);

    return angle;
}