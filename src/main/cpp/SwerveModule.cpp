#include "SwerveModule.h"

SwerveModule::SwerveModule(int angMotorPort, int speedMotorPort, int canCoderPort, double offset, bool inverted) : angleMotor_{angMotorPort, "Drivebase"},
speedMotor_{speedMotorPort, "Drivebase"}, canCoder_{canCoderPort, "Drivebase"}, offset_{offset}
{

    speedMotor_.SetInverted(inverted);

    angleMotor_.SetSelectedSensorPosition(0);
    speedMotor_.SetSelectedSensorPosition(0);

    angleMotor_.SetNeutralMode(NeutralMode::Brake);
    speedMotor_.SetNeutralMode(NeutralMode::Brake);
}

units::meters_per_second_t SwerveModule::talonVelToMps(double vel)
{
    double wheel_radius = 0.05;                      // in meters
    double meters_per_rev = wheel_radius * 2 * M_PI; // wheel circumberence
    double ticks_per_rev = 12650;
    return units::meters_per_second_t{vel / 0.1 * (meters_per_rev / ticks_per_rev)};
}

// TODO: check input modulus of Rotation2d
frc::SwerveModuleState SwerveModule::getState()
{
    frc::SwerveModuleState state; // TODO: can this be made inline?
    state.speed = talonVelToMps(speedMotor_.GetSelectedSensorVelocity());
    state.angle = frc::Rotation2d{units::angle::degree_t{angleMotor_.GetSelectedSensorVelocity() + offset_}};
    return state;
}

frc::SwerveModuleState SwerveModule::getOptState(frc::SwerveModuleState state)
{
    double yaw = frc::InputModulus(canCoder_.GetAbsolutePosition() + offset_, -180.0, 180.0);
    frc::SwerveModuleState opt_state = frc::SwerveModuleState::Optimize(state, units::degree_t(yaw));
    return opt_state;
}

void SwerveModule::setAngMotorVoltage(double volts)
{
    angleMotor_.SetVoltage(units::volt_t{volts});
}

void SwerveModule::setSpeedMotor(double power)
{
    speedMotor_.Set(ControlMode::PercentOutput, power);
}

// 1, 107
// 2, 400
// 3, 666
// 4, 921
// 5, 1180
// 6, 1440p
// 7, 1700
// 8, 1960
// 9, 2230
// 10, 2500
// 11, 2730
// 12, 2855


double SwerveModule::calcDrivePID(double driveSpeed)
{
    double velocity = (driveSpeed * GeneralConstants::MAX_RPM * GeneralConstants::TICKS_PER_ROTATION) / 600;
    double error = velocity - speedMotor_.GetSelectedSensorVelocity();

    // frc::SmartDashboard::PutNumber(id_ + "velocity error", error);

    dIntegralError_ += error * dT_;
    double deltaError = (error - dPrevError_) / dT_;
    dPrevError_ = error;

    if (abs(deltaError) < 40 && error > 100) // TODO get value, also test if needed
    {
        deltaError = 0;
        dIntegralError_ = 0;
    }

    // double radPSec = (driveSpeed * GeneralConstants::MAX_RPM) * 2 * M_PI / 60;
    // double feedForward = radPSec / GeneralConstants::Kv;
    double feedForward = GeneralConstants::MAX_VOLTAGE * driveSpeed;
    // feedForward = 0;

    double power = (dkP_ * error) + (dkI_ * aIntegralError_) + (dkD_ * deltaError) + feedForward;

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

// raw velocity in ticks
double SwerveModule::getVelocity()
{
    return speedMotor_.GetSelectedSensorVelocity();
}

double SwerveModule::getYaw()
{
    return canCoder_.GetAbsolutePosition();
}