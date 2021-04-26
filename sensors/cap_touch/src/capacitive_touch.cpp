
#include "cap_touch/capacitive_touch.hpp"

namespace sgd_sensors
{

Capacitive_Touch::Capacitive_Touch() : Node("capacitive_touch")
{
    this->declare_parameter("port", "/dev/novalue");
    this->declare_parameter("thresh", 2000);
    this->declare_parameter("filter_i", 4);
    this->declare_parameter("msg_regex", "Touch:(\\d*),R:(\\d*),L:(\\d*)");

    std::string serial_topic = this->get_parameter("port").as_string();
    serial_topic = "serial_" + serial_topic.substr(serial_topic.find_last_of("/")+1);

    thresh = this->get_parameter("thresh").as_int(); 
    filter_i_ = this->get_parameter("filter_i").as_int() - 1;   // 0 bis filter_i-1
    for (int i = 0; i < filter_i_; i++)
    {
        fvalues.push_back(this->get_parameter("thresh").as_int());
    }

    // 1x publisher, 1x subscriber
    subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(serial_topic, default_qos,
            std::bind(&Capacitive_Touch::on_msg_received, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sgd_msgs::msg::Touch>("handle_touch", default_qos);

    // message Touch:<time>,R:<value>,L:<value>
    std::cout << this->get_parameter("msg_regex").as_string() << std::endl;
    regex_ = std::regex(this->get_parameter("msg_regex").as_string());
}

Capacitive_Touch::~Capacitive_Touch()
{
    // Destroy
}

void
Capacitive_Touch::on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg_)
{
    if (msg_->msg.find("Touch") == std::string::npos) {return;}     // message is not from touch sensor

    std::smatch matches;
    std::regex_search(msg_->msg, matches, regex_);
    
    int time, r, l;
    if (matches.size() > 3) // ist gut
    {
        time = std::stoi(matches[1]);
        r = std::stoi(matches[2]);
        l = std::stoi(matches[3]);
    } else { return; }

    sgd_msgs::msg::Touch touch_msg;
    touch_msg.header.stamp.sec = floor(time / 1000);
    int fr = filter(r);
    int fl = filter(l);
    touch_msg.has_detected = (fr > thresh) && (fl > thresh);
    touch_msg.value = fr < fl ? fr : fl;
    publisher_->publish(touch_msg);
}

int
Capacitive_Touch::filter(int new_value)
{
    double f = pow(2,filter_i_) / (pow(filter_i_ + 1, 2) - 1) * new_value;

    for (int i = 0; i < filter_i_; i++)
    {
        f += pow(2,i) / (pow(filter_i_+1,2)-1) * fvalues[i];
        fvalues[i] = i+1 < filter_i_ ? fvalues[i+1] : new_value;
    }

    return (int) round(f);
}

}   // namespace sgd_sensors

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sgd_sensors::Capacitive_Touch>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capacitive Touch startup complete");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
