//actually I made the limelight class
#include "Limelight.h"


Limelight::Limelight(){
    network_table = nt::NetworkTableInstance::GetDefault().GetTable(table_name);
    network_table->PutNumber("pipeline", PIPELINE);
}

//moved distance calculator to shooter calc

//Return the X offset to the target
double
Limelight::getXOff(){
    return network_table->GetNumber("tx", 10000.0);
}


//Return the y offset to the target
double
Limelight::getYOff(){
    return network_table->GetNumber("ty", 10000.0);
}


//Check if the limeight sees the target
bool
Limelight::targetAquired(){
    double tv = network_table->GetNumber("tv", 0.0);
    if(tv == 0.0){
        return false;
    } else {
        return true;
    }
}

// get corners
std::vector<LLRectangle> 
// DOES NOT WORK CURRENTLY
Limelight::getCorners() {
    std::vector<double> corners = network_table->GetEntry("tcornxy").GetDoubleArray(std::vector<double>());
    // corners = {75, 181,83, 181,83, 177,75, 177,93, 179,93, 182,102, 184,102, 181,111, 183,111, 186,119, 189,119, 186}; // TODO: get rid of this later (for testing)
    std::vector<LLRectangle> ans = std::vector<LLRectangle>();
    
    // TODO: redo because only outputs top left, bottom right -> get working with custom vision pipeline
    // format this array (vector of vectors of pairs)
    // pairs are coordinates (x, y), vectors represent one rectangle, output holds rectangles
    // center of rectangle is first point in array
    // for (int i = 0; i < corners.size(); i += 8) {
    //     LLRectangle rectVector = LLRectangle();
    //     for (int j = i; j < i + 8; j += 2) {
    //         rectVector.push_back(std::make_pair(corners[j], corners[j+1]));
    //     }
    //     ans.push_back(rectVector);
    // }

    ans = {
        {
            {26, 200}, 
            {30, 198}, {22, 202}, {29, 201}
        }, 
        {
            {82, 203},
            {78, 200}, {78, 203}, {85, 205}, {85, 202}
        }, 
        {
            {65, 197},
            {60, 196}, {60, 199}, {69, 201}, {69, 198}
        },
        {
            {45, 198},
            {40, 196}, {40, 199}, {49, 199}, {49, 196}
        }
    };

    return ans;
}




//coordinates: gonna assume angle is zero when robot facing directly away
frc::Pose2d Limelight::getPose(double navx, double turretAngle) {
    // double distance = getDist() + 0.686;
    // double robotGoalAngle_ = -(turretAngle + getAdjustedX()); 
    // double angleToGoal = navx + robotGoalAngle_;
    // double y = -distance * cos(angleToGoal * M_PI / 180);
    // double x = distance * sin(angleToGoal * M_PI / 180);

   // frc::SmartDashboard::PutNumber("distance", distance);
    // frc::SmartDashboard::PutNumber("robotGoalAngle", robotGoalAngle_);
    // frc::SmartDashboard::PutNumber("angleToGoal", angleToGoal);
    // frc::SmartDashboard::PutNumber("Pose x", x);
    // frc::SmartDashboard::PutNumber("Pose y", y);

  //  return frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{units::degree_t{navx}}};
}

double Limelight::getDist() {
    //do a thing where we get the dist between pose and 0,0
}



//Set the LED mode
void
Limelight::setLEDMode(std::string mode){
    if(mode == "OFF"){
        network_table->PutNumber("ledMode", 1.0);
    }
    if(mode == "BLINK"){
        network_table->PutNumber("ledMode", 2.0);
    }
    if(mode == "ON"){
        network_table->PutNumber("ledMode", 3.0);
    }
}

// Pixels to Angles
// returns the angle from the camera to the pixel
std::pair<double, double> 
Limelight::pixelsToAngle(double px, double py) {
    // From here: https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
    const double H_FOV = 54;
    const double V_FOV = 41;
    const double IMG_WIDTH = 320;
    const double IMG_HEIGHT = 240;

    // normalized x and y
    double nx = (1/(IMG_WIDTH/2)) * (px - 159.5);
    double ny = (1/(IMG_HEIGHT/2)) * (119.5 - py);

    // view plane width/height
    double vpw = 2.0 * tan(H_FOV/2);
    double vph = 2.0 * tan(V_FOV/2);

    // view plane coordinates
    double x = vpw/2 * nx;
    double y = vph/2 * ny;

    // calc angles
    double ax = atan2(1, x); 
    double ay = atan2(1, y);

    std::pair<double, double> ans(ax, ay);
    return ans;
}

// angles to actual x, y, z of point
LL3DCoordinate
Limelight::angleToCoords(double ax, double ay, double targetHeight) {
    // From: https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6
    // and https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7 

    // ax and ay are the angles from the camera to the point 
    // make sure x rotation is around y-axis
    // make sure y rotation is around x-axis
    double x = tan(ax);
    double y = tan(ay);
    double z = 1;

    double length = sqrt(x*x + y*y + z*z);

    // get normalized x, y, and z values
    x = x/length;
    y = y/length;
    z = z/length;

    // apply transformations to 3d vector (compensate for pitch) -> rotate down by camera pitch
    // multiply [x, y, z] vector by rotation matrix around x-axis
    double theta = -GeneralConstants::cameraPitch * M_PI / 180; // for testing
    x = x*1 + y*0 + z*0; // technically not necessary, but just for understandability
    y = x*0 + y*cos(theta) + z*(-sin(theta));
    z = x*0 + y*sin(theta) + z*cos(theta);

    // denormalize coordinates via known height
    double scale = (targetHeight - GeneralConstants::cameraHeight) / y;    

    x *= scale;
    y *= scale;
    z *= scale;

    x += GeneralConstants::cameraHeight;
    y += GeneralConstants::cameraHeight;
    z += GeneralConstants::cameraHeight;
    
    return {x, y, z};
}


//thanks mechanical advantage https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/util/CircleFitter.java
LL3DCoordinate Limelight::center(std::vector<LL3DCoordinate> points, double precision) {
    double xSum = 0.0;
    double ySum = 0.0;
    for (LL3DCoordinate point : points) {
        xSum += point.x;
        ySum += point.z;
    }

    LL3DCoordinate center{xSum / points.size() + GeneralConstants::radius, GeneralConstants::goalHeight, ySum / points.size()};

    //iterate to find optimal center (binary search regression)
    double shiftDist = GeneralConstants::radius / 2.0;
    double minResidual = calcResidual(GeneralConstants::radius, points, center);

    while (true) {
        std::vector<LL3DCoordinate> translations {
            LL3DCoordinate{shiftDist, GeneralConstants::goalHeight, 0.0}, LL3DCoordinate{-shiftDist, GeneralConstants::goalHeight, 0.0},
            LL3DCoordinate{0.0, GeneralConstants::goalHeight, shiftDist}, LL3DCoordinate{0.0, GeneralConstants::goalHeight, -shiftDist}
        };
        LL3DCoordinate bestPoint = center;
        bool centerIsBest = true;

        //check all adjacent positions
        for (LL3DCoordinate translation : translations) {
            LL3DCoordinate c{center.x + translation.x, center.y + translation.y, center.z + translation.z};
            double residual = calcResidual(GeneralConstants::radius, points, c);
            if (residual < minResidual) {
                bestPoint = c;
                minResidual = residual;
                centerIsBest = false;
                break;
            }
        }

        //decrease shift, exit, or continue
        if (centerIsBest) {
            shiftDist /= 2.0;
            if (shiftDist < precision) {
                return center;
            } else {
                center = bestPoint;
            }
        }
    }
}

double Limelight::calcResidual(double radius, std::vector<LL3DCoordinate> points, LL3DCoordinate center) {
    double residual = 0.0;
    for (LL3DCoordinate point : points) {
        double distance = sqrt((point.x-center.x)*(point.x-center.x) + (point.y-center.y)*(point.y-center.y) + (point.z-center.z)*(point.z-center.z));
        double diff = distance - radius;
        residual += diff * diff;
    }
    return residual;
}

// relative to x-axis
int angleBetween(const LLCoordinate point, const LLCoordinate centerPoint) {
    int angleA = atan2(centerPoint.second - point.second, centerPoint.first - point.first) * 180 / M_PI;
    angleA = (angleA + 360) % 360;

    return angleA;
}

// relative to y-axis
int angleBetweenY(const LLCoordinate point, const LLCoordinate centerPoint) {
    int angleA = atan2(centerPoint.first - point.first, centerPoint.second - point.second) * 180 / M_PI;

    return angleA;
}

struct AngleComparator {
    LLCoordinate centerPoint;
    AngleComparator(LLCoordinate centerPoint_) : centerPoint(centerPoint_) {};

    bool operator ()(const LLCoordinate& a, const LLCoordinate& b) {
        return angleBetween(a, centerPoint) > angleBetween(b, centerPoint);
    }
};

struct SortingCorner {
    LLCoordinate corner;
    int angle;
    int index;
    bool set = false;

    SortingCorner(int index, LLCoordinate corner, LLCoordinate centerPoint) {
        this->index = index;
        this->corner = corner;
        this->angle = angleBetweenY(corner, centerPoint);
        // std::cout << "(" << corner.first << ", " << corner.second << ") - angle: " << this->angle << "\n";
    }
};

bool sortSortingCornersAbs(const SortingCorner& a, const SortingCorner& b) {
    return abs(a.angle) < abs(b.angle);
}

LLRectangle
Limelight::sortCorners(LLRectangle rectCorners) {
    if (rectCorners.empty()) {
        return rectCorners;
    }

    // lower bound threshhold for what counts as a "top" corner
    int TOP_THRESHHOLD = 90;
    if (rectCorners.size() == 5) { // full corner set -> looser restrictions
        TOP_THRESHHOLD = 180;
    }

    // assume centerPoint is first point in rectCorners arr
    const LLCoordinate centerPoint = rectCorners[0];

    // rectCorners is a vector with 4 pairs -> each pair is a coordinate (x, y)
    int topLeftIndex = -1;
    int topRightIndex = -1;
    std::vector<LLCoordinate> ans(4, {-1, -1});

    std::vector<SortingCorner> sortingCorners = std::vector<SortingCorner>();
    for (int i = 1; i < rectCorners.size(); i++) {
        sortingCorners.push_back(SortingCorner(i, rectCorners[i], centerPoint));
    }

    // sort sorting corners by absolute value
    sort(sortingCorners.begin(), sortingCorners.end(), sortSortingCornersAbs);

    SortingCorner highestCorner = sortingCorners[0];
    if (abs(highestCorner.angle) <= TOP_THRESHHOLD) {
        highestCorner.set = true;

        // check if top left or top right
        // top left
        if (highestCorner.angle < 0) {
            topLeftIndex = highestCorner.index;
        }
            // top right
        else {
            topRightIndex = highestCorner.index;
        }
    }

    // search corners for missing top corner
    int counter = 2;
    for (int i = 1; i < sortingCorners.size(); i++) {
        // sorting corner already found
        if (sortingCorners[i].set) {
            continue;
        }

        if (abs(sortingCorners[i].angle) <= TOP_THRESHHOLD) {
            // only search angles < 0 b/c top left not found yet
            if (topLeftIndex == -1) {
                if (sortingCorners[i].angle < 0) {
                    topLeftIndex = sortingCorners[i].index;
                    sortingCorners[i].set = true;
                    continue;
                }
            }
                // only search angles > 0 b/c top right not found yet
            else if (topRightIndex == -1) {
                if (sortingCorners[i].angle > 0) {
                    topRightIndex = sortingCorners[i].index;
                    sortingCorners[i].set = true;
                    continue;
                }
            }
        }

        // if not top left/right, set to back corners
        if (counter <= 3) {
            ans[counter] = rectCorners[sortingCorners[i].index];
            counter ++;
        }
        else {
            std::cout << "Something went wrong... more than 2 corners on bottom\n";
        }
    }


    // should have top left and top right corners by now (set them)
    if (topLeftIndex != -1) {
        ans[0] = rectCorners[topLeftIndex];
    }
    if (topRightIndex != -1) {
        ans[1] = rectCorners[topRightIndex];
    }

    // output: corners are sorted in following order: [topLeft, topRight, bottomLeft, bottomRight]
    // or [top1, top2, bottom1, bottom2]
    return ans;
}

std::vector<LL3DCoordinate> 
Limelight::getCoords() {
    std::vector<LLRectangle> corners = getCorners();

    std::vector<LL3DCoordinate> coords = std::vector<LL3DCoordinate> ();

    for (int i = 0; i < corners.size(); i++) {
        // TODO: get this to work
        // corners[i] = sortCorners(corners[i]);

        // if (corners[i].size() != 4) {
        //     std::cout << "Something went wrong... rectangle array corners is: " << corners[i].size();
        // }

        for (int j = 0; j < corners[i].size(); j++) {
            std::pair<double, double> anglePair = pixelsToAngle(corners[i][j].first, corners[i][j].second);
            coords.push_back(
                angleToCoords(
                    anglePair.first, 
                    anglePair.second, 
                    j < 2 ? GeneralConstants::targetHeightUpper : GeneralConstants::targetHeightLower
                )
            );
        }
    }

    return coords;
}