/*
 *  Some text
 */

# include "sgd_lc/wh_f-cruiser.hpp"

namespace sgd_motor
{

using namespace std::chrono_literals;

WH_Fcruiser::WH_Fcruiser() : Node("wh_cruiser")
{
    pub_motor_ = this->create_publisher<sgd_msgs::msg::Serial>("write_ttyUSB0",default_qos);
    timer_ = this->create_wall_timer(1000ms, std::bind(&WH_Fcruiser::publish_motordata, this));
    message_count_ = 0;

    // 2x publisher, 2x subscriber
}

WH_Fcruiser::~WH_Fcruiser()
{
    // Destroy
}

void
WH_Fcruiser::publish_motordata()
{
    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    std::string m = "Hello Ardu - message count: ";
    m.append(std::to_string(message_count_++));
    msg.msg = m;

    pub_motor_->publish(msg);
}

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sgd_motor::WH_Fcruiser>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor driver startup completed.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
