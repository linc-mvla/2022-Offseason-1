#pragma once

#include <iostream>
#include <math.h>
#include "Constants.h"
#include <frc/geometry/Pose2d.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <frc/smartdashboard/SmartDashboard.h>


class Limelight{
    public:
        Limelight();

        double getXOff();
        double getYOff();
        double getAdjustedX();
        bool hasTarget();

        void adjustAngles(double& x, double& y);
        double calcDistance();

        frc::Pose2d getPose(double navx, double turretAngle);

        void lightOn(bool light);
        std::shared_ptr<nt::NetworkTable> GetNetworkTable();

    private:
        void ReadPeriodicIn();

        std::shared_ptr<nt::NetworkTable> table;
        std::string tableName = "limelight";

        //const int PIPELINE = 0;
};