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

TEST_F(LimelightTest, CoordsTest) {
  std::vector<LL3DCoordinate> coords = limelight_.getCoords();
  for (LL3DCoordinate c : coords) {
    std::cout << "(" << c.x << ", " << c.z << ", " << c.y << ")" << "\n";
  } 
  std::cout << "\n";
}

// TEST_F(LimelightTest, AngleTest) {
//   LL3DCoordinate c = limelight_.angleToCoords(-8 * M_PI / 180, 20 * M_PI / 180, GeneralConstants::targetHeightUpper);
//   std::cout << "angleToCoords test\n";
//   std::cout << c.x << ", " << c.y << ", " << c.z << "\n";
// }

TEST_F(LimelightTest, CenterTest) {
  std::vector<LL3DCoordinate> coords = limelight_.getCoords();
  LL3DCoordinate center = limelight_.getCenter(coords, 0.01);
  std::cout << "center: " << center.x << ", " << center.y << ", " << center.z << "\n";
}



int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
