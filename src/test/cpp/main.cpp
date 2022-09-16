#include <hal/HAL.h>
#include "Limelight.h"
#include "gtest/gtest.h"

class LimelightTest : public testing::Test {
  protected:
    Limelight limelight_;
};

TEST_F(LimelightTest, madTest) {

  limelight_.getCorners({4, 248, 148, 248, 150, 251, 151, 251, 148, 249, 149, 1, 147, 37, -1, -1, 3, 166, 24, 157, 30, 165, 27, 161, 26, 4, 202, 19, 203, 22, 213, 22, 213, 19, 207, 20, 3, 178, 23, 189, 19, 178, 20, 183, 20});
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
