#include "AutoPaths.h"

AutoPaths::AutoPaths()/* : swervePath_(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV)*/
{
    pathNum_ = 0;
}

void AutoPaths::setPath(Path path)
{
    path_ = path;
    pathNum_ = 0;
    swervePaths_.clear();

    switch(path_)
    {
        case TAXI_DUMB:
        {
            break;
        }
        case TWO_DUMB: 
        {

        }
        case TWO_RIGHT:
        {
            SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p1.addPoint(SwervePose(0, 0, 90, 0));
            p1.addPoint(SwervePose(1, 0, 90, 0)); //TODO get value
            p1.addPoint(SwervePose(0, 0, 90, 0));
            
            swervePaths_.push_back(p1);
            break;
        }
        case TWO_MIDDLE:
        {
            SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p1.addPoint(SwervePose(0, 0, 135, 0));
            p1.addPoint(SwervePose(0.7, -0.7, 135, 0)); //TODO get value
            p1.addPoint(SwervePose(0, 0, 135, 0));
            
            swervePaths_.push_back(p1);
            break;
        }
        case TWO_LEFT:
        {
            SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p1.addPoint(SwervePose(0, 0, -135, 0));
            p1.addPoint(SwervePose(-0.7, -0.7, -135, 0)); //TODO get value
            p1.addPoint(SwervePose(0, 0, -135, 0));
            
            swervePaths_.push_back(p1);
            break;
        }
        case THREE:
        {
            SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p1.addPoint(SwervePose(0, 0, 90, 0));
            p1.addPoint(SwervePose(1, 0, 90, 0)); //TODO get value
            p1.addPoint(SwervePose(0, 0, 90, 0));

            SwervePath p2(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p2.addPoint(SwervePose(0, 0, 90, 0));
            p2.addPoint(SwervePose(-2, -2, -135, 1.5));
            
            swervePaths_.push_back(p1);
            swervePaths_.push_back(p2);
            break;
        }
        case BIG_BOY:
        {
            SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p1.addPoint(SwervePose(0, 0, 90, 0));
            p1.addPoint(SwervePose(1, 0, 90, 0)); //TODO get value
            p1.addPoint(SwervePose(0, 0, 90, 0));

            SwervePath p2(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p2.addPoint(SwervePose(0, 0, 90, 0));
            p2.addPoint(SwervePose(-2, -2, -135, 1.5));

            SwervePath p3(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p3.addPoint(SwervePose(-2, -2, -135, 1.5));
            p3.addPoint(SwervePose(1, -4, 135, 2));

            SwervePath p4(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

            p4.addPoint(SwervePose(1, -4, 135, 2));
            p4.addPoint(SwervePose(-2, -2, 135, 0));
            
            swervePaths_.push_back(p1);
            swervePaths_.push_back(p2);
            swervePaths_.push_back(p3);
            swervePaths_.push_back(p4);
            break;
        }
    }
}


AutoPaths::Path AutoPaths::getPath()
{
    return path_;
}

Shooter::State AutoPaths::getShooterState()
{
    return shooterState_;
}

Intake::State AutoPaths::getIntakeState()
{
    return intakeState_;
}

void AutoPaths::startTimer()
{
    startTime_ = timer_.GetFPGATimestamp().value();
}

/*void AutoPaths::stopTimer()
{
    timer_.Stop();
}*/

void AutoPaths::periodic(double yaw, SwerveDrive* swerveDrive)
{
    double time = timer_.GetFPGATimestamp().value() - startTime_;

    bool pathsOver = false;
    if(path_ != TAXI_DUMB && path_ != TWO_DUMB)
    {
        bool endOfSwervePath = false;
        //SwervePose* pose;
        for(size_t i = pathNum_; i < swervePaths_.size(); ++i) //Maybe make a while loop idk
        {
            //pose = &swervePaths_[i].getPose(time, endOfSwervePath);

            if(i == swervePaths_.size() - 1 && endOfSwervePath)
            {
                pathsOver = true;
            }

            if(endOfSwervePath && i != swervePaths_.size() - 1)
            {
                pathNum_++;
                startTimer();
                break;
            }
        }

        //swerveDrive->drivePose(yaw, *pose);

        //delete pose;
    }
    

    switch(path_)
    {
        case TAXI_DUMB:
        {
            intakeState_ = Intake::RETRACTED_IDLE;
            shooterState_ = Shooter::IDLE;
            if(timer_.Get().value() < 2.0) //TODO get values
            {
                swerveDrive->drive(0, 0.2, 0);
            }
            else
            {
                swerveDrive->drive(0, 0, 0);
            }
            break;
        }
        case TWO_DUMB:
        {
            if(timer_.Get().value() < 2.0) //TODO get values
            {
                intakeState_ = Intake::INTAKING;
                shooterState_ = Shooter::TRACKING;
                swerveDrive->drive(0, 0.2, 0);
            }
            else
            {
                swerveDrive->drive(0, 0, 0);
                shooterState_ = Shooter::REVING;
            }
            break;
        }
        case TWO_RIGHT:
        {
            intakeState_ = Intake::INTAKING;
            if(pathsOver)
            {
                shooterState_ = Shooter::REVING;
            }
            else
            {
                shooterState_ = Shooter::TRACKING;
            }
            break;
        }
        case TWO_MIDDLE:
        {
            intakeState_ = Intake::INTAKING;
            if(pathsOver)
            {
                shooterState_ = Shooter::REVING;
            }
            else
            {
                shooterState_ = Shooter::TRACKING;
            }
            break;
        }
        case TWO_LEFT:
        {
            intakeState_ = Intake::INTAKING;
            if(pathsOver)
            {
                shooterState_ = Shooter::REVING;
            }
            else
            {
                shooterState_ = Shooter::TRACKING;
            }
            break;
        }
        case THREE:
        {
            break;
        }
        case BIG_BOY:
        {
            break;
        }

    }
}

double AutoPaths::initYaw()
{
    switch(path_)
    {
        case TAXI_DUMB:
        {
            return 0;
            break;
        }
        case TWO_DUMB:
        {
            return 0;
            break;
        }
        case TWO_RIGHT:
        {
            return 90;
            break;
        }
        case TWO_MIDDLE:
        {
            return 135;
            break;
        }
        case TWO_LEFT:
        {
            return -135;
            break;
        }
        case THREE:
        {
            return  90;
            break;
        }
        case BIG_BOY:
        {
            return 90;
            break;
        }
    }

    return 0;
}