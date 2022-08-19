#include <hal/HAL.h>

#include "gtest/gtest.h"
#include "Limelight.h"

class LimelightTest : public testing::Test {
  protected:
    Limelight limelight_;
};

TEST_F(LimelightTest, InputTest) {
  std::vector<LLRectangle> rects = limelight_.getCorners();
  std::cout << "num rects: " << rects.size() << "\n";
  for (LLRectangle r : rects) {
    for (LLCoordinate c : r) std::cout << c.first << "," << c.second << " ";
    std::cout << "\n";
  }
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
