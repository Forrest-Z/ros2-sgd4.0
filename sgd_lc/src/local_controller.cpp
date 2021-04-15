
#include "sgd_lc/local_controller.hpp"

namespace sgd_lc
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Local_Controller::Local_Controller() : Node("local_controller")
{
    // Declare parameters from launch file
    this->declare_parameter("port", "/dev/novalue");

    std::string port = this->get_parameter("port").as_string();
    std::string topic_pub = "write_" + port.substr(port.find_last_of("/")+1);
    std::string topic_sub = "serial_" + port.substr(port.find_last_of("/")+1);

    pub_motor_ = this->create_publisher<sgd_msgs::msg::Serial>(topic_pub, default_qos); 
    sub_motor_ = this->create_subscription<sgd_msgs::msg::Serial>(topic_sub, default_qos,
        std::bind(&Local_Controller::print, this, _1));
    sub_touch_ = this->create_subscription<sgd_msgs::msg::Touch>("handle_touch", default_qos,
        std::bind(&Local_Controller::on_touch_received, this, std::placeholders::_1));
    sub_laser_ = this->create_subscription<sensor_msgs::msg::Range>("laser_1d", default_qos,
        std::bind(&Local_Controller::on_laser_received, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&Local_Controller::publish_motordata, this));
    speed_ = 0.0;
}

Local_Controller::~Local_Controller()
{
    // Destroy
}

void
Local_Controller::publish_motordata()
{
    if (!last_handle_ || last_range_ > 60 || last_range_ < 2)
    {
        speed_ = 0;
    } else {
        double set_speed = MAX_SPEED - MAX_SPEED / 60.0 * last_range_;
        double difference = set_speed - speed_;     // Abweichung
        speed_ = speed_ + difference * 0.1;
    }   

    // publish data
    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    msg.msg = std::to_string(round(speed_));

    std::cout << "Publish new speed: " << round(speed_) << std::endl;
    pub_motor_->publish(msg);
}

void
Local_Controller::print(const sgd_msgs::msg::Serial::SharedPtr msg_)
{
    std::string m = msg_->msg;
    //std::cout << "T: " << now().nanoseconds() << "; S: " << speed_ << "; msg: " << m << std::endl;
}

void
Local_Controller::on_laser_received(const sensor_msgs::msg::Range::SharedPtr msg_)
{
    last_range_ = msg_->range;
}

void
Local_Controller::on_touch_received(const sgd_msgs::msg::Touch::SharedPtr msg_)
{
    last_handle_ = msg_->has_detected;
}

}   // namespace sgd_lc

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sgd_lc::Local_Controller>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor driver startup completed.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
