#pragma once

#include "Constants.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

class Channel
{
    public:
        enum Color
        {
            BLUE,
            RED,
            UNKNOWN
        };
        void setColor(Color color);
        Color getColor();

        Channel();
        void periodic();

        int getBalls();
        bool badIdea();
        int getBallCount();
        int getBallsShot();
        void increaseBallCount();
        void decreaseBallCount();
        void setBallCount(int ballCount);
        void setBallsShot(int ballsShot);
        void setSeeingBall(bool seeingBall);

    private:

        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 colorSensor_{i2cPort};

        int ballCount_, ballsShot_;
        bool seeingBall_;

        Color color_;
};