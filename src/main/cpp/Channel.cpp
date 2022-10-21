#include "Channel.h"

Channel::Channel(Intake* intake)
{
    intake_ = intake;
    ballsShot_ = 0;
    seeingBall_ = false;
    /*
    frc::SmartDashboard::PutNumber("Red Hue", 0.0);
    frc::SmartDashboard::PutNumber("Blue Hue", 0.0);
    frc::SmartDashboard::PutNumber("Max Range Blue", 0.0);
    frc::SmartDashboard::PutNumber("Max Range Red", 0.0);
    frc::SmartDashboard::PutNumber("Red Hue", 0.0);
    frc::SmartDashboard::PutNumber("Red Hue", 0.0);
    */
}

void Channel::periodic()
{
    int proximity = colorSensor_.GetProximity();
    //frc::SmartDashboard::PutNumber("prox", proximity);
    int ballCount = balls_.size();
    while(balls_[0].timer.HasElapsed(ChannelConstants::BALLEXITTIME)){//If a ball's timer expires, it has been shot
        shotBall();
    }
    //Checking what it sees (ballz)
    if(!seeingBall_ && proximity > ChannelConstants::BALL_PROXIMITY){//When it changes to a ball seen
        addChannelBall();
        seeingBall_ = true;
    }
    if(seeingBall_ && proximity < ChannelConstants::BALL_PROXIMITY){//When it changes to no ball seen
        switch(intake_->getState()){
            case Intake::State::INTAKING:
                balls_[ballCount-1].timer.Start();//Set the last ball to loading
                balls_[ballCount-1].state = Ball::State::KICKER;
                break;
            case Intake::State::LOADING://No clue if this is possible (also why)
                balls_.pop_back();//Ball exits
                break;
            case Intake::State::OUTAKING:
                balls_.pop_back();//Ball exits
                break;
            default:
                break;
        }
        seeingBall_ = false;
    }
    frc::SmartDashboard::PutNumber("Ball Count", ballCount);
}

Channel::Ball Channel::getNextBall(){
    return balls_[0];
}

void Channel::addChannelBall(){
    //Assume proximity is correct (checked in periodic)
    frc::Color color = colorSensor_.GetColor();
    //frc::SmartDashboard::PutNumber("r", color.red);
    //frc::SmartDashboard::PutNumber("g", color.green);
    //frc::SmartDashboard::PutNumber("b", color.blue);
    Color ballColor;
    if(color.red > 1.5 * color.blue){
        ballColor = RED;
    }
    else if(color.blue > 1.5 * color.red){
        ballColor = BLUE;
    }
    else{
        ballColor = UNKNOWN;
    }
    //0.394, 0.1856, red
    //0.5003, 0.1207, red
    //0.3907, 0.1861, red
    //0.2625, 0.2432, neither

    /*
    //https://www.rapidtables.com/convert/color/rgb-to-hsv.html
    //Convert to HSV for some nicer reads
    //Might change to HSL
    double r, g, b;
    r = color.red;
    g = color.green;
    b = color.blue;
    double Cmax = std::max({r,g,b});
    double Cmin = std::min({r,g,b});
    double delta = Cmax - Cmin;
    
    double h;//Hue is in degrees, basically color 0 = red, 180 = blueish, 360 = red
    if(delta == 0.0){
        h = 0;
    }
    else{
        if(Cmax == r){
            h = 60.0 * std::fmod((g-b)/delta,6);
        }
        else if(Cmax == g){
            h = 60.0 * (((b-r)/delta) + 2.0);
        }
        else if(Cmax == b){
            h = 60.0 * (((r-g)/delta) + 4.0);
        }
        else{h = 0;}//If some BS happens
    }
    double s;
    if(Cmax == 0.0){
        s = 0;
    }
    else if (Cmax != 0.0){
        s = delta/Cmax;
    }
    else{s = 0;}//If some BS happens
    double v = Cmax;
    
    frc::SmartDashboard::PutNumber("h", h);
    frc::SmartDashboard::PutNumber("s", s);
    frc::SmartDashboard::PutNumber("v", v);

    double redHue = frc::SmartDashboard::GetNumber("Red Hue", 0.0);
    double blueHue = frc::SmartDashboard::GetNumber("Blue Hue", 0.0);
    double maxRangeRed = frc::SmartDashboard::GetNumber("Max Range Blue", 0.0);
    double maxRangeBlue = frc::SmartDashboard::GetNumber("Max Range Red", 0.0);
    double minValue = frc::SmartDashboard::GetNumber("Red Hue", 0.0);
    double minSat = frc::SmartDashboard::GetNumber("Red Hue", 0.0);
    if(s < minSat || v < minValue){
        ballColor = UNKNOWN;
    }
    else{
        double rDist = std::fmod(h-redHue, 180.0);
        double bDist = std::fmod(h-blueHue, 180.0);
        bool isRed = r < maxRangeRed;
        bool isBlue = b < maxRangeBlue;
        if(isRed && !isBlue){
            ballColor == RED;
        }
        else if(isBlue && !isRed){
            ballColor == BLUE;
        }
        else{
            ballColor = UNKNOWN;
        }
    }
    */
    frc::Timer t;
    t.Stop();
    Ball b = {ballColor, Ball::State::CHANNEL, t};
    balls_.push_back(b);
}

void Channel::setKickerDirection(int direction){
    if(direction != kickerDirection_){
        for(int i = 0; i<getBallCount(); i++){
            if(balls_[i].state == Ball::State::KICKER){
                if(direction > 0){//Moving forward
                    balls_[i].timer.Start();
                }
                else if(direction == 0){
                    balls_[i].timer.Stop();
                }
                else{//Negative direction
                    balls_[i].timer.Reset();
                    balls_[i].timer.Stop();
                }
            }
        }
    }
}

bool Channel::isBallGood(){
    return balls_[0].color == color_;
}

int Channel::getBallCount(){
    return balls_.size();
}

void Channel::shotBall(){
    balls_.pop_front();
    ballsShot_ += 1;
}

void Channel::clearBalls(){
    balls_.clear();
}

void Channel::setColor(Color color)
{
    color_ = color;
}

Channel::Color Channel::getColor()
{
    return color_;
}

<<<<<<< HEAD
/**
 * @brief Checks if there is a ball that is our color
 * 
 * @return true - There is correct ball
 * @return false - There is no ball/wrong color
 */
bool Channel::badIdea()
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
=======
int Channel::getBallsShot(){
>>>>>>> channelWithQueue
    return ballsShot_;
}

void Channel::setBallsShot(int ballsShot)
{
    ballsShot_ = ballsShot;
}

void Channel::setSeeingBall(bool seeingBall)
{
    seeingBall_ = seeingBall;
}