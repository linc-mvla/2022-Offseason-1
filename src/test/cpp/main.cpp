#include <hal/HAL.h>

#include "gtest/gtest.h"  

#include "Intake.h"
#include "Channel.h"

class ChannelTest : public testing::Test{
  protected:
    Intake intake;
    Channel channel;
}

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
