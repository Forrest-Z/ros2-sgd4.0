
#include "laser_1d/laser_1d.hpp"

namespace sgd_sensors
{

Laser_1D::Laser_1D() : Node("laser_1d")
{
    this->declare_parameter("port", "/dev/novalue");
    this->declare_parameter("msg_regex", "Laser:(\\d*),R:(\\d*),S:(\\d*),P:(\\d*\\.\\d*),A:(\\d*\\.\\d*)");

    std::string serial_topic = this->get_parameter("port").as_string();
    serial_topic = "serial_" + serial_topic.substr(serial_topic.find_last_of("/")+1);

    // 1x publisher, 1x subscriber
    subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(serial_topic, default_qos,
            std::bind(&Laser_1D::on_msg_received, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("laser_1d", default_qos);

    // message: Laser: <time>,R: <range>,S: <status>,P: <peak_count_rate>,A: <ambient_count_rate>
    regex_ = std::regex(this->get_parameter("msg_regex").as_string());
}

Laser_1D::~Laser_1D()
{
    // Destroy
}

void
Laser_1D::on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg_)
{
    // {Laser: <time>,R: <range>,S: <status>,P: <peak_count_rate>,A: <ambient_count_rate>}
    // Parse message and save values
    // publish values
    if (msg_->msg.find("Laser") == std::string::npos) {return;}     // message is not from laser sensor

    std::smatch matches;
    std::regex_search(msg_->msg, matches, regex_);
    
    int time, r;//, s, p, a;
    if (matches.size() > 3) // ist gut
    {
        time = std::stoi(matches[1]);
        r = std::stoi(matches[2]);
        //s = std::stoi(matches[3]);    // currently unused
        //p = std::stoi(matches[4]);
        //a = std::stoi(matches[5]);
    } else { return; }

    sensor_msgs::msg::Range laser_msg;
    laser_msg.header.stamp.sec = floor(time / 1000);
    laser_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    laser_msg.min_range = 0.0f;
    laser_msg.max_range = 150.0f;
    laser_msg.range = r / 10.0f;
    publisher_->publish(laser_msg);

}

}   // namespace sgd_sensors

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sgd_sensors::Laser_1D>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Laser 1D startup complete");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
