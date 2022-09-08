#include "SwerveDrive.h"

SwerveDrive::SwerveDrive(AHRS * nx, Limelight* limelight): navx_{nx}, limelight_{limelight}
{
    angPID_.EnableContinuousInput(-180, 180);
    angPID_.SetIntegratorRange(-0.5, 0.5);
}

void SwerveDrive::initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose) {
  delete odometry_;
  odometry_ = new frc::SwerveDriveOdometry<4>(m_kinematics, gyroAngle, initPose);
}

void SwerveDrive::setYaw(double yaw)
{
    yaw_ = yaw;
}

//returns an object that contains the speed and angle of each swerve module
wpi::array<frc::SwerveModuleState, 4> SwerveDrive::getRealModuleStates() {
  wpi::array<frc::SwerveModuleState, 4> moduleStates = {
    flModule_.getState(), frModule_.getState(), blModule_.getState(), brModule_.getState() };
  return moduleStates;
}

void SwerveDrive::Periodic(units::meters_per_second_t dx, units::meters_per_second_t dy, units::radians_per_second_t dtheta, 
units::degree_t navx_yaw, double turretAngle) {
  //converts field-relative joystick input to robot-relative speeds
  speeds_ = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    dx, dy, dtheta, frc::Rotation2d(navx_yaw));

  //convert robot speeds into swerve module states (speed & angle of each module)
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds_);
  
  //update odometry
  odometry_->Update(navx_yaw, getRealModuleStates());
  lPose = limelight_->getPose(navx_->GetYaw(), turretAngle);
  if (!(abs(lPose.X().value()) > 10 || abs(lPose.Y().value()) > 10 || (frc::Timer::GetFPGATimestamp().value() - limelight_->getLastUpdated()) >= 35)) {
      updateLimelightOdom(turretAngle, false);
  }


  //optimize module states so they can have most direct path to goal
  auto fl_opt = flModule_.getOptState(fl);
  auto fr_opt = frModule_.getOptState(fr);
  auto bl_opt = blModule_.getOptState(bl);
  auto br_opt = brModule_.getOptState(br);

 //TODO: fix weird thing with speed if drivers complain or we have time
  flModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(flModule_.getYaw(), fl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  flModule_.setSpeedMotor( 0.2*std::clamp(fl_opt.speed.value(), -1.0, 1.0) );

  frModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(frModule_.getYaw(), fr_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  frModule_.setSpeedMotor( 0.2*std::clamp(fr_opt.speed.value(), -1.0, 1.0) );

  blModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(blModule_.getYaw(), bl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  blModule_.setSpeedMotor( 0.2*std::clamp(bl_opt.speed.value(), -1.0, 1.0) );

  brModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(brModule_.getYaw(), br_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  brModule_.setSpeedMotor( 0.2*std::clamp(br_opt.speed.value(), -1.0, 1.0) );


}

frc::ChassisSpeeds SwerveDrive::getRobotSpeeds() {
  frc::ChassisSpeeds speeds = m_kinematics.ToChassisSpeeds(getRealModuleStates());
  //rest of stuff would be for conversion to field-relative speeds
//   frc::Translation2d t{units::meter_t{speeds.vx.value()}, units::meter_t{speeds.vy.value()}}; //sus units... if we hate it i can write this myself
//   t.RotateBy(frc::Rotation2d{units::degree_t{m_navx->GetYaw()}});
//   speeds.vx = units::meters_per_second_t{t.X().value()};
//   speeds.vy = units::meters_per_second_t{t.Y().value()};
  return speeds;
}

double SwerveDrive::getDistance(double turretAngle)
{
        /*if (!limelight_->hasTarget())
    {
        return -1;
    }*/
    //could just use limelightDist, but i want to take advantage of averaging for smoothness
    std::pair<double, double> robotLimelight = camToBot(turretAngle); //so that 
    
    double limelightToGoalX = odometry_->GetPose().X().value() + robotLimelight.first;
    double limelightTtoGoalY = odometry_->GetPose().Y().value() + robotLimelight.second;
    return sqrt(limelightToGoalX * limelightToGoalX + limelightTtoGoalY * limelightTtoGoalY);
}

std::pair<double, double> SwerveDrive::camToBot(double turretAngle) {

    double turretLimelightAngle = turretAngle - 180;

    frc::InputModulus(turretLimelightAngle, -180.0, 180.0);
    turretLimelightAngle = turretLimelightAngle * M_PI / 180;
    double turretLimelightX = LimelightConstants::TURRET_CENTER_RADIUS * sin(turretLimelightAngle);
    double turretLimelightY = LimelightConstants::TURRET_CENTER_RADIUS * cos(turretLimelightAngle);

    turretLimelightY -= LimelightConstants::ROBOT_TURRET_CENTER_DISTANCE;

    double angle = navx_->GetYaw();
    double robotLimelightX = turretLimelightX * cos(angle) - turretLimelightY * sin(angle);
    double robotLimelightY = turretLimelightX * sin(angle) + turretLimelightY * cos(angle);

    return {robotLimelightX, robotLimelightY};    
}

void SwerveDrive::updateLimelightOdom(double turretAngle, bool inAuto)
{
    //get limelight pose (so this is the position of the camera)
        limelightX_ = lPose.X().value();
        limelightY_ = lPose.Y().value();        

        //adjust to get position of actual robot
       std::pair<double, double> trans = camToBot(turretAngle);

        limelightX_ -= trans.first;
        limelightY_ -= trans.second;

        units::meter_t averagedX = units::meter_t{0.04*limelightX_ + 0.96*odometry_->GetPose().X().value()};
        units::meter_t averagedY = units::meter_t{0.04*limelightY_ + 0.96*odometry_->GetPose().Y().value()};
        frc::Rotation2d rot = frc::Rotation2d{units::degree_t{navx_->GetYaw()}};

        updateOdometry(rot, frc::Pose2d{averagedX, averagedY, rot});
}

/*void SwerveDrive::resetGoalOdometry(double turretAngle)
{
    if(!limelight_->hasTarget())
    {
        return;
    }

    foundGoal_ = true;
    goalY_ = - (limelight_->calcDistance() + 0.6096); //center of goal is origin
    goalX_ = 0;
    yawOffset_ = -yaw_ - (180 - (turretAngle + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET));
}*/

void SwerveDrive::reset()
{
    limelightX_ = 0;
    limelightY_ = 0;
    //reset odometry
    //goalX_ = 0;
    //goalY_ = 0;
    //yawOffset_ = 0;
    foundGoal_ = false;
}

double SwerveDrive::getRobotGoalAng()
{
    /*if(foundGoal_)
    {
        
        double robotGoalAng = -yaw_ - yawOffset_;

        robotGoalAng += 360 * 10;
        robotGoalAng = ((int)floor(robotGoalAng) % 360) + (robotGoalAng - floor(robotGoalAng));
        robotGoalAng -= 360 * floor(robotGoalAng / 360 + 0.5);

        return robotGoalAng;
    }
    else
    {
        return 0;
    }*/
    
    return robotGoalAngle_;
}

bool SwerveDrive::foundGoal()
{
    return foundGoal_;
}

void SwerveDrive::setFoundGoal(bool foundGoal)
{
    foundGoal_ = foundGoal;
}

double SwerveDrive::getX()
{
    return odometry_->GetPose().X().value();
}

double SwerveDrive::getY()
{
    return odometry_->GetPose().Y().value();
}

double SwerveDrive::getSmoothX()
{
    return smoothX_;
}

double SwerveDrive::getSmoothY()
{
    return smoothY_;
}

double SwerveDrive::getSWX()
{
    return smoothWheelX_;
}

double SwerveDrive::getSWY()
{
    return smoothWheelY_;
}

/*double SwerveDrive::getGoalX()
{
    return goalX_;
}

double SwerveDrive::getGoalY()
{
    return goalY_;
}*/

double SwerveDrive::getGoalXVel() //TODO implement limelight distance if math works?
{
    /*if(limelight_->calcDistance() != -1)
    {
        return goalXVel_;
    }

    if(goalY_ == 0 && goalX_ ==0)
    {
        return 0;
    }
    double angToGoal = (atan2(-goalY_, -goalX_) * 180 / M_PI);

    double rGoalXVel = goalXVel_ * cos(angToGoal) + goalYVel_ * -sin(angToGoal);
    frc::SmartDashboard::PutNumber("RGXV", rGoalXVel);
    return rGoalXVel;*/

    frc::SmartDashboard::PutNumber("RGXV", goalXVel_);
    return goalXVel_;

}

double SwerveDrive::getGoalYVel()
{
    /*if(limelight_->calcDistance() != -1)
    {
        return goalYVel_;
    }

    if(goalY_ == 0 && goalX_ ==0)
    {
        return 0;
    }
    double angToGoal = (atan2(-goalY_, -goalX_) * 180 / M_PI);

    double rGoalYVel = goalXVel_ * sin(angToGoal) + goalYVel_ * cos(angToGoal);
    frc::SmartDashboard::PutNumber("RGYV", rGoalYVel);
    return rGoalYVel;*/

    frc::SmartDashboard::PutNumber("RGYV", goalYVel_);
    return goalYVel_;
}