#include <hal/HAL.h>

#include "gtest/gtest.h"
#include "Limelight.h"

class LimelightTest : public testing::Test {
  protected:
    Limelight limelight_;
};

// TEST_F(LimelightTest, InputTest) {
//   std::vector<LLRectangle> rects = limelight_.getCorners();
//   std::cout << "num rects: " << rects.size() << "\n";
//   for (LLRectangle r : rects) {
//     for (LLCoordinate c : r) std::cout << c.first << "," << c.second << " ";
//     std::cout << "\n";
//   }
// }

//currently i just change images by changing llpython in getCorners
//doing this bc i don't want to mess with main code too much? may change if I use this more

// TEST_F(LimelightTest, CoordsTest) {
//   std::vector<LL3DCoordinate> coords = limelight_.getCoords();
//   for (LL3DCoordinate c : coords) {
//     std::cout << "(" << c.x << ", " << c.z << ", " << c.y << ")" << "\n";
//   } 
//   std::cout << "\n";
// }

// TEST_F(LimelightTest, AngleTest) {
//   LL3DCoordinate c = limelight_.angleToCoords(-8 * M_PI / 180, 20 * M_PI / 180, GeneralConstants::targetHeightUpper);
//   std::cout << "angleToCoords test\n";
//   std::cout << c.x << ", " << c.y << ", " << c.z << "\n";
// }

// TEST_F(LimelightTest, angleToCoordsTest) {
//   LL3DCoordinate c = limelight_.angleToCoords(-15.88 * M_PI / 180, 6.17 * M_PI / 180, 0.655);
//   std::cout << c.x << ", " << c.z << ", " << c.y << "\n";
// }

// TEST_F(LimelightTest, pixelsToAnglesTest) {
//   std::pair<double, double> p = limelight_.pixelsToAngle(79, 91);
//   std::cout << p.first * 180 / M_PI << ", " << p.second * 180 / M_PI << "\n";
// }

TEST_F(LimelightTest, leastSquaresTest) {
  std::vector<LL3DCoordinate> points = {
    
  {0.0666092, 2.641, 2.13232},
  {0.00435346, 2.641, 2.07901},
  {0.0300805, 2.5902, 2.06723},
  {0.108157, 2.5902, 2.085},
  {0.106006, 2.5902, 2.03238},
  {-0.177332, 2.641, 2.06171},
  {-0.227754, 2.641, 2.04462},
  {-0.112989, 2.5902, 1.99845},
  {-0.410891, 2.641, 2.06171},
  {-0.442616, 2.641, 2.04462},
  {-0.436746, 2.5902, 2.03238},
  {-0.347336, 2.5902, 1.99845},
  {-0.34285, 2.5902, 1.9654}
 
};
  
  LL3DCoordinate c = limelight_.getCenter(points);
  std::cout << "center: " << c.x << ", " << c.z << ", " << c.y << "\n";
}

// std::pair<double, double> genPixels(double x, double z) {
//   Eigen::Vector3d L{-x, GeneralConstants::cameraHeight, -z}; //treated as a point but needs to be vector for rotation later
//   Eigen::Vector3d H{0, GeneralConstants::targetHeightUpper, 0}; //can change if we're doing full gen

//   //so for the limelight's plane of view, the line would go through L in the direction of H
// }

// TEST_F(LimelightTest, fullTest) {
//   std::vector<double> llpython;
//   LL3DCoordinate center = limelight_.getCenter(limelight_.getCoords(llpython));
//   std::cout << "center x: " << center.x << "\n";
//   std::cout << "center y: " << center.z << "\n";
// }

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
