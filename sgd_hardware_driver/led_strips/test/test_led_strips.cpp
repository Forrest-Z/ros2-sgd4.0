#include "led_node.hpp"
#include "sgd_msgs/msg/light.hpp"
#include "sgd_msgs/msg/serial.hpp"

#include "gtest/gtest.h"

TEST(LED_Strip, ComputeOutput)
{
    sgd_msgs::msg::Light light1;
    light1.mode = sgd_msgs::msg::Light::FILL;
    light1.strip = sgd_msgs::msg::Light::LEFT;

    std::vector<unsigned char> rgb = {0,0,0};
    light1.rgb = rgb;

    EXPECT_EQ(sgd_hardware_drivers::LED_Strip::compute_msg(*light1).msg, "L0,1,0,0,0");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}