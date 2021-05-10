#include "sgd_util/serial.hpp"

namespace sgd_util
{

using namespace std::chrono_literals;   // if a timer is used

Serial::Serial():
    nav2_util::LifecycleNode("example_node", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    rec_state_ = RECEIVER_STATUS::INITIAL;

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("logfile", rclcpp::ParameterValue("serial.log")); // mock-file
    add_parameter("baud_rate", rclcpp::ParameterValue(9600));
    add_parameter("read_write", rclcpp::ParameterValue("rw"));
    add_parameter("raw", rclcpp::ParameterValue(false));
    add_parameter("sframe", rclcpp::ParameterValue("$"));
    add_parameter("log", rclcpp::ParameterValue(false));
}

Serial::~Serial()
{
    // Destroy
}

nav2_util::CallbackReturn
Serial::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    time_at_start_ = round(now().seconds() * 1000);
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();
    
    rec_state_ = open_port(port_.c_str(), baud_rate_);
    rec_state_ = set_config();    

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Serial::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    pub_serial->on_activate();
    // TODO authentification if requested
    rec_state_ = raw_ ? IN_MSG : WAIT_HEADER;

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Serial::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    pub_serial->on_deactivate();

    // TODO terminate connection
    rec_state_ = close_port();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Serial::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    pub_serial.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Serial::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Serial::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("logfile", logfile_);
    get_parameter("baud_rate", baud_rate_);
    get_parameter("read_write", read_write_);
    //get_parameter("raw", raw_);
    //get_parameter("sframe", sframe_);
    get_parameter("log", log_);
}

void
Serial::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init publisher and subscriber");
    
    std::string topic_out = "serial_" + port_.substr(port_.find_last_of("/")+1);
    pub_serial = create_publisher<sgd_msgs::msg::Serial>(topic_out, default_qos);

    timer_ = this->create_wall_timer(10ms, std::bind(&Serial::read_serial, this));

    if (this->get_parameter("read_write").as_string() == "rw")
    {
        // Create subscription -> mock currently does not support writing operations
        std::string subsc_name = "write_" + port_.substr(port_.find_last_of("/")+1);
        sub_serial = this->create_subscription<sgd_msgs::msg::Serial>(subsc_name,default_qos,
                std::bind(&Serial::write_serial, this, std::placeholders::_1));
    }
}

Serial::RECEIVER_STATUS
Serial::open_port(const char *port, const int baud)
{
    // Open file
    try
    {
        file_.open(logfile_, std::ios::in);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "File could not be openend: %s", logfile_.c_str());
        return RECEIVER_STATUS::STOPPED;
    }
    
    return RECEIVER_STATUS::ESTABLISH;
}

void
Serial::read_serial()
{
    if (rec_state_ < RECEIVER_STATUS::WAIT_HEADER)
    {
        RCLCPP_ERROR(get_logger(), "Serial communication is not configured");
        return;
    }

    if (read_buf_.empty())
    {
        getline(file_, read_buf_);
    }

    if (file_.eof())
    {
        rclcpp::shutdown();
        return;   // end of logfile reached
    }

    try
    {
        //std::string st = read_buf_.substr(0, read_buf_.find(","));
        //RCLCPP_INFO(get_logger(), "String to long: %s", st.c_str());
        long t = std::stol(read_buf_.substr(0, read_buf_.find(",")));  // get time from line

        if (round(now().seconds() * 1000) - time_at_start_ > t)
        {
            sgd_msgs::msg::Serial ser_msg;
            ser_msg.header.stamp = now();
            ser_msg.port = port_;
            ser_msg.msg = read_buf_.substr(read_buf_.find(",")+1);

            pub_serial->publish(ser_msg);
            read_buf_.clear();
        }
    }
    catch(const std::invalid_argument& e)
    {
        RCLCPP_WARN(get_logger(), "Error parsing string to long");
        return;
    }
}

void
Serial::write_serial(const sgd_msgs::msg::Serial::SharedPtr msg_)
{
    RCLCPP_DEBUG(get_logger(), "Serial write is not implemented in serial mock.");
}

Serial::RECEIVER_STATUS
Serial::set_config()
{
    // Send config,wait for config-ack or config-nak
    // If authentification requested return RECEIVER_STATUS::AUTHENTIFICATE
    return RECEIVER_STATUS::WAIT_HEADER;
}

Serial::RECEIVER_STATUS
Serial::close_port()
{
    // Send terminate request, wait for terminate-ack or -nak
    file_.close();
    return RECEIVER_STATUS::STOPPED;
}


}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_util::Serial>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
