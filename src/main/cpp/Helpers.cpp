#include "Helpers.h"

void Helpers::normalizeAngle(double& angle) //Need work + Clarity
{//Basically makes the angle within the range of [0, 360]
    //TODO use mod or ternary expressions
    angle += 360 * 10;
    angle = ((int)floor(angle) % 360) + (angle - floor(angle));
    angle -= 360 * floor(angle / 360 + 0.5);
}

//TODO include PID?
//TODO RGB->HSV? (channel)