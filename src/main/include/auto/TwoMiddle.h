#pragma once
#include "AutoState.h"

class TwoMiddle {
    TwoMiddle();
    SwervePath makePath();

    private:
        SwervePath path{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};
        AutoState drivingBack_;
        AutoState intaking_;
        AutoState shooting_;
};