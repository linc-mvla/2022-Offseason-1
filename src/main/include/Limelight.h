#pragma once

#include <iostream>
#include <math.h>
#include "Constants.h"
#include <frc/geometry/Pose2d.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/EntryListenerFlags.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define M_PI 3.14159265358979323846

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

        double lastUpdated = 0;

        std::shared_ptr<nt::NetworkTable> table;
        std::string tableName = "limelight";

        //const int PIPELINE = 0;
};