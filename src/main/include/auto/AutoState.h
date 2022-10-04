#pragma once
#include "Shooter.h"
#include "Intake.h"
#include "Channel.h"
#include "Constants.h"
#include "SwervePath.h"

//could be a struct but stackoverflow told me it's bad practice to have virtual functions in a struct
class AutoState {
    public:
        AutoState();
        AutoState(SwervePath path);
        SwervePath swervePath_{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};
        Shooter::State shooterState_;
        Intake::State intakeState_;

        virtual bool isFinished();

};
