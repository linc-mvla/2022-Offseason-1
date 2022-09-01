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
    {1.32, 2.64, 5.91}, {1.16, 2.64, 5.91}, {1.3, 2.64, 5.84}, {1.18, 2.64, 5.93}, {2.25, 2.64, 5.65}, {2.14, 2.64, 5.65}, {2.23, 2.64, 5.58}, {2.11, 2.64, 5.58}, 
    {1.59, 2.64, 5.57}, {1.5, 2.64, 5.57}, {1.59, 2.64, 5.58}, {1.44, 2.64, 5.67}, {1.89, 2.64, 5.5}, {1.78, 2.64 ,5.5}, {1.74, 2.64, 5.42}, {1.92, 2.64, 5.5}  
  };
  LL3DCoordinate c = limelight_.getCenter(points);
  std::cout << c.x << ", " << c.z << ", " << c.y << "\n";
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
