#pragma once

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h>
#include <iostream>
#include <frc/MathUtil.h>
#include <frc/system/LinearSystem.h>

#define M_PI 3.14159265358979323846

namespace GeneralConstants{ 
    const double goalHeight = 2.641;
    const double targetHeightUpper = 2.641;
    const double targetHeightLower = targetHeightUpper - 0.0508;
    const double cameraHeight = 1.36;//1.384; //1.19
    const double cameraPitch = 24;//22;
    const double radius = 0.6096;
}

typedef std::pair<double, double> LLCoordinate;
typedef std::vector<LLCoordinate> LLRectangle;
typedef std::pair<double, double> LLAnglePair;

struct LL3DCoordinate {
    double x;
    double y;
    double z;
};

class Limelight{
    public:

        Limelight();

        double getXOff();
        double getYOff();
        bool hasTarget();
        void lightOn(bool light);

        double getDist(double navx, double turretAngle);
        frc::Pose2d getPose(double navx, double turretAngle);
        double getAdjustedX();
        std::shared_ptr<nt::NetworkTable> GetNetworkTable();


    private:
        void ReadPeriodicIn();

        std::vector<double> getLLPython();

        std::shared_ptr<nt::NetworkTable> network_table;
        std::string table_name = "limelight";

        std::vector<LLRectangle> getCorners(std::vector<double> llpython);
        LLCoordinate pixelsToAngle(double px, double py);
        LL3DCoordinate angleToCoords(double ax, double ay, double targetHeight); 
        LLRectangle sortCorners(LLRectangle rectCorners);
        std::vector<LL3DCoordinate> getCoords(std::vector<double> llpython);
        LL3DCoordinate getCenter(std::vector<LL3DCoordinate> points);
        LL3DCoordinate estimateCenter(std::vector<LL3DCoordinate> points);
        
};