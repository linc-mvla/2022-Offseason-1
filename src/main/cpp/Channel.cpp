#include "Channel.h"

Channel::Channel()
{
    ballCount_ = 0;
    ballsShot_ = 0;
    seeingBall_ = false;
}

void Channel::periodic()
{
    int proximity = colorSensor_.GetProximity();
    if(!seeingBall_ && proximity > ChannelConstants::BALL_PROXIMITY)
    {
        increaseBallCount();
        seeingBall_ = true;
    }

    if(seeingBall_ && proximity < ChannelConstants::BALL_PROXIMITY)
    {
        seeingBall_ = false;
    }

    frc::SmartDashboard::PutNumber("Ball Count", ballCount_);

    if(ballCount_ == 0 && proximity > ChannelConstants::BALL_PROXIMITY)
    {
        ballCount_ = 1;
    }
    //see if this happens enough to merit this clause
}

void Channel::setColor(Color color)
{
    color_ = color;
}

Channel::Color Channel::getColor()
{
    return color_;
}

bool Channel::badIdea()//Checks if there is a ball that is our color
{
    frc::Color color = colorSensor_.GetColor(); //Get color of color sensor
    int proximity = colorSensor_.GetProximity(); //Get distance of color sensor

    //frc::SmartDashboard::PutNumber("prox", proximity);
    //frc::SmartDashboard::PutNumber("r", color.red);
    //frc::SmartDashboard::PutNumber("g", color.green);
    //frc::SmartDashboard::PutNumber("b", color.blue);

    Color ballColor; //Ball color variable
    if(color.red > 1.5 * color.blue) //If it is 1.5x more red than blue
    {
        ballColor = RED; //Set ballcolor to red
    }
    else if(color.blue > 1.5 * color.red) //if it is 1.5x more blue than red
    {
        ballColor = BLUE; //Set ballcolor to blue
    }
    else
    {
        ballColor = UNKNOWN; //Set unknown ballcolor
    }

    //0.394, 0.1856, red
    //0.5003, 0.1207, red
    //0.3907, 0.1861, red
    //0.2625, 0.2432, neither

    if(proximity < ChannelConstants::BALL_PROXIMITY) //if the sensor detects something too far away
    {
        return false; //Does not find ball
    }

    //TODO probably redundant
    bool badIdea = (ballColor != color_ && ballColor != UNKNOWN); //Check ball is not our alliance color or unknown

    return badIdea;//Return true/false
}

int Channel::getBallCount()
{
    return ballCount_;
}

int Channel::getBallsShot()
{
    return ballsShot_;
}

void Channel::increaseBallCount()
{
    if(ballCount_ < GeneralConstants::MAX_BALL_COUNT)
    {
        ++ballCount_;
    }
}

void Channel::decreaseBallCount()
{
    if(ballCount_ > 0)
    {
        --ballCount_;
        ++ballsShot_;
    }
}

void Channel::setBallCount(int ballCount)
{
    if(ballCount < 0)
    {
        ballCount_ = 0;
    }
    else if(ballCount > GeneralConstants::MAX_BALL_COUNT)
    {
        ballCount_ = GeneralConstants::MAX_BALL_COUNT;
    }
    else
    {
        ballCount_ = ballCount;
    }
}

void Channel::setBallsShot(int ballsShot)
{
    ballsShot_ = ballsShot;
}

void Channel::setSeeingBall(bool seeingBall)
{
    seeingBall_ = seeingBall;
}