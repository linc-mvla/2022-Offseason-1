#include "AutoPaths.h"

AutoPaths::AutoPaths(Channel *channel) : channel_(channel)
{
    pathNum_ = 0;
    dumbTimerStarted_ = false;
}

void AutoPaths::setPath(Path path)
{
    path_ = path;
    pathNum_ = 0;
    swervePaths_.clear();
    nextPathReady_ = false;
    dumbTimerStarted_ = false;

    switch (path_)
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
        p1.addPoint(SwervePose(1, 0, 90, 0)); // TODO get value
        p1.addPoint(SwervePose(0, 0, 90, 0));

        swervePaths_.push_back(p1);
        break;
    }
    case TWO_MIDDLE:
    {
        SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

        p1.addPoint(SwervePose(0, 0, 135, 0));
        p1.addPoint(SwervePose(0.7, -0.7, 135, 0)); // TODO get value
        p1.addPoint(SwervePose(0, 0, 135, 0));

        swervePaths_.push_back(p1);
        break;
    }
    case TWO_LEFT:
    {
        SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

        p1.addPoint(SwervePose(0, 0, -135, 0));
        p1.addPoint(SwervePose(-0.7, -0.7, -135, 0)); // TODO get value
        p1.addPoint(SwervePose(0, 0, -135, 0));

        swervePaths_.push_back(p1);
        break;
    }
    case THREE:
    {
        SwervePath p1(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);

        p1.addPoint(SwervePose(0, 0, 90, 0));
        p1.addPoint(SwervePose(1, 0, 90, 0)); // TODO get value
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
        p1.addPoint(SwervePose(1, 0, 90, 0)); // TODO get value
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

void AutoPaths::periodic(double yaw, SwerveDrive *swerveDrive)
{
    double time = timer_.GetFPGATimestamp().value() - startTime_;

    bool pathsOver = false;
    bool endOfSwervePath = false;
    if (path_ != TAXI_DUMB && path_ != TWO_DUMB)
    {
        SwervePose *pose;
        for (size_t i = pathNum_; i < swervePaths_.size(); ++i) // Maybe make a while loop idk
        {
            pose = swervePaths_[i].getPose(time, endOfSwervePath);
            if (!endOfSwervePath)
            {
                break;
            }

            if (i == swervePaths_.size() - 1 && endOfSwervePath)
            {
                pathsOver = true;
            }

            if (nextPathReady_ && endOfSwervePath && i != swervePaths_.size() - 1)
            {
                nextPathReady_ = false;
                pathNum_++;
                startTimer();
                time = timer_.GetFPGATimestamp().value() - startTime_;
                break;
            }
        }

        swerveDrive->drivePose(yaw, *pose);

        if(pose != nullptr)
        {
            delete pose;
        }
        
    }
    else
    {
        if (!dumbTimerStarted_)
        {
            timer_.Start();
        }
    }

    switch (path_)
    {
    case TAXI_DUMB:
    {
        intakeState_ = Intake::RETRACTED_IDLE;
        shooterState_ = Shooter::IDLE;
        if (timer_.Get().value() < 2.0) // TODO get values
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
        intakeState_ = Intake::INTAKING;
        if (timer_.Get().value() < 2.0) // TODO get values
        {
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
        if (pathsOver)
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
        if (pathsOver)
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
        if (pathsOver)
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
        intakeState_ = Intake::INTAKING;
        if (endOfSwervePath && (channel_->getBallsShot() < 2 || channel_->getBallCount() > 0))
        {
            shooterState_ = Shooter::REVING;
        }
        else
        {
            shooterState_ = Shooter::TRACKING;
        }

        if (pathNum_ == 0 && endOfSwervePath)
        {
            if (!failsafeStarted_)
            {
                failsafeStarted_ = true;
                failsafeTimer_.Start();
            }

            if (failsafeTimer_.Get().value() > 5 || channel_->getBallsShot() > 1)
            {
                failsafeTimer_.Stop();
                channel_->setBallsShot(0);
                nextPathReady_ = true;
            }
        }
        break;
    }
    case BIG_BOY:
    {
        intakeState_ = Intake::INTAKING;
        if (!endOfSwervePath)
        {
            shooterState_ = Shooter::TRACKING;
        }
        else
        {
            if(pathNum_ != 2)
            {
                shooterState_ = Shooter::REVING;
            }
            else
            {
                shooterState_ = Shooter::TRACKING;
            }
            switch (pathNum_)
            {
            case 0:
            {
                if (!failsafeStarted_)
                {
                    failsafeStarted_ = true;
                    failsafeTimer_.Start();
                }

                if (failsafeTimer_.Get().value() > 3 || channel_->getBallsShot() > 1)
                {
                    failsafeTimer_.Stop();
                    channel_->setBallsShot(0);
                    nextPathReady_ = true;
                }
                break;
            }
            case 1:
            {
                if (!failsafeStarted_)
                {
                    failsafeStarted_ = true;
                    failsafeTimer_.Start();
                }

                if (failsafeTimer_.Get().value() > 2 || channel_->getBallsShot() > 0)
                {
                    failsafeTimer_.Stop();
                    channel_->setBallsShot(0);
                    nextPathReady_ = true;
                }
                break;
            }
            case 2:
            {
                if (!failsafeStarted_)
                {
                    failsafeStarted_ = true;
                    failsafeTimer_.Start();
                }

                if (failsafeTimer_.Get().value() > 2)
                {
                    failsafeTimer_.Stop();
                    nextPathReady_ = true;
                }
                break;
            }
            case 3:
            {
                shooterState_ = Shooter::REVING;
                break;
            }
            }
        }
        break;
    }
    }
}

double AutoPaths::initYaw()
{
    switch (path_)
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
        return 90;
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