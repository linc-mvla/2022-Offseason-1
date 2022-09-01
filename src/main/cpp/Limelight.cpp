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
Limelight::getCorners() {
    //REMEMBER TO COMMENT NEXT 2 LINES AFTER TESTING
   // nt::NetworkTableEntry e = network_table->GetEntry("llpython");
  //  std::vector<double> llpython = e.GetDoubleArray({});

    //REMEMBER TO COMMENT OUT WHEN NOT TESTING

    //first image
    // std::vector<double> llpython = {
    //     4, 114,167, 111,168, 112,170, 115,169,  113,168, 
    //     3, 154,165, 160,167, 160,165,  157,165, 
    //     3, 138,166, 145,164, 139,164,  140,164, 
    //     4, 129,165, 128,164, 124,165, 124,167,  126,165
    // };
  

    //second image
    // std::vector<double> llpython = {
    //     3, 225, 147, 231, 144, 227, 144, 227, 145, 
    //     4, 250, 136, 243, 138, 242, 141, 249, 139, 246, 138, 
    //     3, 273, 131, 262, 134, 273, 134, 268, 133, 
    //     4, 286, 132, 296, 132, 296, 130, 290, 129, 291, 130
    // };

    //first image 2nd set
    // std::vector<double> llpython = {
    //     4, 177, 191, 176, 192, 176, 193, 177, 193, 176, 192, 
    //     3, 122, 194, 127, 192, 123, 191, 124, 192, 3, 163, 190, 170, 192, 169, 189, 166, 190, 
    //     4, 149, 188, 149, 191, 156, 190, 156, 188, 152, 189, 
    //     4, 141, 189, 134, 190, 134, 192, 141, 191, 137, 190
    // };

    //second image 2nd set
    std::vector<double> llpython = {
        4, 229, 176, 221, 176, 222, 179, 229, 178, 225, 177, 
        4, 277, 173, 277, 175, 284, 175, 283, 173, 279, 173, 
        4, 239, 176, 248, 175, 248, 172, 243, 172, 244, 173, 
        5, 259, 173, 260, 175, 268, 174, 266, 171, 260, 171, 263, 172
    };

    //start with num corners, then corners, then center, repeat
    //have to do a wierd system cause network tables can't handle outputting fancy data types like 2d arrays
    int corner = 0;
    int numCorners = 0;
    bool formingRect = false;
    std::vector<LLRectangle> rectangles = {};
    LLRectangle tempRectangle;

    for(int i = 0; i < llpython.size()-1; i++){
        double p = llpython[i];
        if (!formingRect) { //starting new rect
            numCorners = llpython[i];
            tempRectangle.clear();
            corner = 0;
            formingRect = true;
        } else { //continuing curr rect
            if (corner < numCorners) { //getting points of rect
                tempRectangle.push_back(LLCoordinate{llpython[i], llpython[i+1]});
            } else { //getting last element, the center
                tempRectangle.insert(tempRectangle.begin(), LLCoordinate{llpython[i], llpython[i+1]});
                formingRect = false;
                LLRectangle copy = tempRectangle; //copies by default in c++
                rectangles.push_back(copy); 
            }
            corner++;
            i++;
        }
    }
   
    return rectangles;
    
}


//coordinates: gonna assume angle is zero when robot facing directly away
frc::Pose2d Limelight::getPose(double navx, double turretAngle) {
    LL3DCoordinate center = getCenter(getCoords());

    frc::SmartDashboard::PutNumber("Center x", center.x);
    frc::SmartDashboard::PutNumber("Center y", center.z);

    frc::Translation2d pose{units::meter_t{-center.x}, units::meter_t{-center.z}};
    
    double turretLimelightAngle = turretAngle - 180;
    frc::InputModulus(turretLimelightAngle, -180.0, 180.0);

    frc::SmartDashboard::PutNumber("Turret limelight angle", turretLimelightAngle);

    pose.RotateBy(frc::Rotation2d{units::degree_t{navx + turretLimelightAngle}});

    frc::SmartDashboard::PutNumber("Pose x", pose.X().value());
    frc::SmartDashboard::PutNumber("Pose y", pose.Y().value());

    return frc::Pose2d{pose.X(), pose.Y(), frc::Rotation2d{units::degree_t{navx}}};
}

//hub coords are (0, 0)
double Limelight::getDist(double navx, double turretAngle) {
    frc::Pose2d pose = getPose(navx, turretAngle); 
    return sqrt(pose.X().value()*pose.X().value() + pose.Y().value()*pose.Y().value());
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
    const double H_FOV = 59.6; //54
    const double V_FOV = 49.7;
    const double IMG_WIDTH = 320;
    const double IMG_HEIGHT = 240;

    // normalized x and y
    double nx = (1/(IMG_WIDTH/2)) * (px - 159.5);
    double ny = (1/(IMG_HEIGHT/2)) * (119.5 - py);

    // view plane width/height
    double vpw = 2.0 * tan(H_FOV/2 * M_PI / 180);
    double vph = 2.0 * tan(V_FOV/2 * M_PI / 180);

    // view plane coordinates
    double x = vpw/2 * nx;
    double y = vph/2 * ny;

    // std::cout << "vp coords: " << x << ", " << y << "\n";

    // calc angles
    double ax = atan2(x, 1); 
    double ay = atan2(y, 1);

    // std::cout << "pixels vs angles: " << px << ", " << py << " :: " << ax << ", " << ay << "\n";

    std::pair<double, double> ans(ax, ay);
    return ans;
}

LL3DCoordinate Limelight::getCenter(std::vector<LL3DCoordinate> points) {
    //https://goodcalculators.com/best-fit-circle-least-squares-calculator/
    
    //step 1: find A and B in Ax=B
    double a00, a01, a02, a10, a11, a12, a20, a21, a22;
    a22 = points.size();
    double b0, b1, b2;
    for (LL3DCoordinate c : points) {
        a00 += c.x*c.x;
        a01 += c.x*c.z;
        a02 += c.x;

        a10 += c.x*c.z;
        a11 += c.z*c.z;
        a12 += c.z;

        a20 += c.x;
        a21 += c.z;

        b0 += c.x*(c.x*c.x + c.z*c.z);
        b1 += c.z*(c.x*c.x + c.z*c.z);
        b2 += c.x*c.x + c.z*c.z;
    }

    Eigen::Matrix3d A;
    A(0, 0) = a00; A(0, 1) = a01; A(0, 2) = a02;
    A(1, 0) = a10; A(1, 1) = a11; A(1, 2) = a12;
    A(2, 0) = a20; A(2, 1) = a21; A(2, 2) = a22;

    Eigen::Vector3d B;
    B(0) = b0; B(1) = b1; B(2) = b2;

    //step 2: least squares. compute x = (A_t*A)^-1 * A_t*b
    //https://textbooks.math.gatech.edu/ila/least-squares.html
    Eigen::Vector3d x = (A.transpose() * A).inverse() * (A.transpose()*B);

    //step 3: transform x0 & x1 into k & m in circle equation
    //can also compute radius but don't need that for now
    double k = x(0) / 2;
    double m = x(1) / 2;

    return {k, 2.641, m};
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

    // apply transformations to 3d vector (compensate for pitch) -> rotate down by camera pitch
    // multiply [x, y, z] vector by rotation matrix around x-axis
    double theta = -GeneralConstants::cameraPitch * M_PI / 180; // convert to radians
    double newX = x*1 + y*0 + z*0; // technically not necessary, but just for understandability
    double newY = x*0 + y*cos(theta) + z*(-sin(theta));
    double newZ = x*0 + y*sin(theta) + z*cos(theta);

    x = newX; y = newY; z = newZ;

    // denormalize coordinates via known height
    double scale = (targetHeight - GeneralConstants::cameraHeight) / y;    

    x *= scale;
    y *= scale;
    z *= scale;

    y += GeneralConstants::cameraHeight;
    
    return {x, y, z};
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
        corners[i] = sortCorners(corners[i]);

        // if (corners[i].size() != 4) {
        //     std::cout << "Something went wrong... rectangle array corners is: " << corners[i].size();
        // }

        for (int j = 0; j < corners[i].size(); j++) {
            if (corners[i][j].first == -1 || corners[i][j].second == -1) {
                continue;
            }
         //   std::cout << "corner: (" << corners[i][j].first << ", " << corners[i][j].second << ")\n";
            std::pair<double, double> anglePair = pixelsToAngle(corners[i][j].first, corners[i][j].second);
            std::cout << "angle x: " << anglePair.first * 180 / M_PI << ", angle y: " << anglePair.second * 180 / M_PI << "\n";
            coords.push_back(
                angleToCoords(
                    anglePair.first, 
                    anglePair.second, 
                    j < 2 ? GeneralConstants::targetHeightUpper : GeneralConstants::targetHeightLower
                )
            );
         //   std::cout << "coordinate: (" << coords[coords.size()-1].x << ", " << coords[coords.size()-1].y << ", " << coords[coords.size()-1].z << ")\n";
        }
    }

    for (int i = 0; i < coords.size(); i++) {
        std::vector<double> temp = {coords[i].x, coords[i].y, coords[i].z};
        frc::SmartDashboard::PutNumberArray("coords " + i, temp);
    }

    return coords;
}