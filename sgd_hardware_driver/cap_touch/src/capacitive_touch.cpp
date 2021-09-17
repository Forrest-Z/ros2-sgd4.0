#include "cap_touch/capacitive_touch.hpp"

namespace sgd_sensors
{

Capacitive_Touch::Capacitive_Touch():
    nav2_util::LifecycleNode("capacitive_touch", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("thresh", rclcpp::ParameterValue(2000));
    add_parameter("filter_i", rclcpp::ParameterValue(4));
    add_parameter("msg_regex", rclcpp::ParameterValue("Touch:(\\d*),R:(\\d*),L:(\\d*)"));

}

Capacitive_Touch::~Capacitive_Touch()
{
    // Destroy
}

nav2_util::CallbackReturn
Capacitive_Touch::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    regex_ = std::regex(msg_regex_);

    for (int i = 0; i < filter_i_-1; i++)
    {
        fvalues.push_back(this->get_parameter("thresh").as_int());
    }

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Capacitive_Touch::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    publisher_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Capacitive_Touch::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Capacitive_Touch::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publisher_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Capacitive_Touch::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Capacitive_Touch::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("thresh", thresh_);
    get_parameter("filter_i", filter_i_);
    get_parameter("msg_regex", msg_regex_);
}

void
Capacitive_Touch::init_pub_sub()
{
    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);

    subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(serial_topic, default_qos,
            std::bind(&Capacitive_Touch::on_msg_received, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sgd_msgs::msg::Touch>("handle_touch", default_qos);

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on topic %s",
            serial_topic.c_str(), 'handle_touch');
}

void
Capacitive_Touch::on_msg_received(const sgd_msgs::msg::Serial::SharedPtr msg)
{
    if (msg->msg.find("Touch") == std::string::npos) {return;}     // message is not from touch sensor

    std::smatch matches;
    std::regex_search(msg->msg, matches, regex_);
    
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
    touch_msg.has_detected = (fr > thresh_) && (fl > thresh_);
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

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_sensors::Capacitive_Touch>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
