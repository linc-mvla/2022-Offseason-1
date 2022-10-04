#include "auto/TwoMiddle.h"

SwervePath TwoMiddle::makePath() {
    SwervePath path(SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV);
    path.addPoint(SwervePose(0, 0, 135, 0));
    path.addPoint(SwervePose(0.707, -0.707, 135, 0));
    path.generateTrajectory(false);
    return path;
}

TwoMiddle::TwoMiddle() : drivingBack_(makePath()) {

}
