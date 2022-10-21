#pragma once

#include "Constants.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "Intake.h"

class Channel
{
    public:
        Channel(Intake* intake);
        void periodic();
        enum Color{
            BLUE,
            RED,
            UNKNOWN
        };
        struct Ball{
            enum State{
                KICKER,
                CHANNEL
            };
            Color color;
            State state;
            frc::Timer timer;
        };
        Ball getNextBall();
        void addChannelBall(); //Adds the ball in the channel
        void setKickerDirection(int direction);//Positive for forward, negative for backwards, zero for paused
        bool isBallGood(); //Return if the next ball is the correct color

        int getBallCount();
        void shotBall();
        void clearBalls();

        void setColor(Color c);
        Color getColor();

        int getBallsShot();
        void setBallsShot(int ballsShot);
        void setSeeingBall(bool seeingBall);

    private:
        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 colorSensor_{i2cPort};
        std::deque<Ball> balls_ = {};
        bool seeingBall_;
        int ballsShot_;
        Intake* intake_;
        int kickerDirection_ = 0;
        Color color_; //Color of team
};