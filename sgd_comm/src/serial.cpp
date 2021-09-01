#include "sgd_comm/serial.hpp"

namespace sgd_util
{

using namespace std::chrono_literals;   // if a timer is used

Serial::Serial():
    nav2_util::LifecycleNode("serial_comm", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    rec_state_ = RECEIVER_STATUS::INITIAL;

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("logfile", rclcpp::ParameterValue("serial.log"));
    add_parameter("baud_rate", rclcpp::ParameterValue(9600));
    add_parameter("read_write", rclcpp::ParameterValue("rw"));
    add_parameter("raw", rclcpp::ParameterValue(false));
    add_parameter("sframe", rclcpp::ParameterValue("$"));
    add_parameter("stframe", rclcpp::ParameterValue("%"));
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

    if (log_)
    {
        file_.open(logfile_, std::ios::out | std::ios::trunc);
    }
    
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
    if (file_.is_open())
    {
        file_.close();
    }
    
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
    get_parameter("raw", raw_);
    std::string s1;
    get_parameter("sframe", s1);
    sframe_ = s1.front();
    RCLCPP_INFO(get_logger(), "Set startframe to %c", sframe_);
    std::string s2;
    get_parameter("stframe", s2);
    stframe_ = s2.front();
    RCLCPP_INFO(get_logger(), "Set stopframe to %c", stframe_);
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
        // Create subscription
        std::string subsc_name = "write_" + port_.substr(port_.find_last_of("/")+1);
        sub_serial = this->create_subscription<sgd_msgs::msg::Serial>(subsc_name,default_qos,
                std::bind(&Serial::write_serial, this, std::placeholders::_1));
    }
}

Serial::RECEIVER_STATUS
Serial::open_port(const char *port, const int baud)
{
    struct termios tty_;

    serial_port_ = open(port, O_RDWR);

    if (serial_port_ < 0)
    {
        RCLCPP_ERROR(get_logger(), "Error %i from open: %s", errno, strerror(errno));
        return RECEIVER_STATUS::STOPPED;
    }

    if (tcgetattr(serial_port_, &tty_) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        RCLCPP_ERROR(get_logger(), "Check all connections and try again.");
        return RECEIVER_STATUS::STOPPED;       // TODO: Error handling
    }

// Control settings
    //tty_.c_cflag &= ~PARENB;     // clear parity bit
    //tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= CS8;
    tty_.c_cflag &= ~CRTSCTS;
    tty_.c_cflag |= CLOCAL;
    //tty_.c_cflag |= (CREAD | CLOCAL);

// Input settings
    //tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_.c_iflag &= ~IXON;
    tty_.c_iflag |= IGNBRK;
    tty_.c_iflag &= ~BRKINT;
    tty_.c_iflag &= ~ICRNL;
    tty_.c_iflag &= ~IMAXBEL;
    //tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

// Output settings
    tty_.c_oflag &= ~OPOST;
    tty_.c_oflag &= ~ONLCR;

// Local settings
    tty_.c_lflag &= ~ECHO;
    tty_.c_lflag &= ~ECHOE;
    tty_.c_lflag &= ~ECHOK;
    tty_.c_lflag &= ~ECHOCTL;
    tty_.c_lflag &= ~ECHOKE;
    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~IEXTEN;
    tty_.c_lflag &= ~ISIG;
    tty_.c_lflag |= NOFLSH;

    tty_.c_cc[VTIME] = 0;
    tty_.c_cc[VMIN] = 0;

    if (baud > 9600)
    {
        cfsetispeed(&tty_, B115200);      // TODO: use user defined baud-rate
        cfsetospeed(&tty_, B115200);
    } else {
        cfsetispeed(&tty_, B9600);      // TODO: use user defined baud-rate
        cfsetospeed(&tty_, B9600);
    }

    if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
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
    
    // read serial port
    char b [512];
    std::memset(&b, '\0', sizeof(b));

    // read one char per read
    int num_bytes;
    char c;
    int i = 0;
    while ((num_bytes = read(serial_port_, &c, 1)) > 0 && i < 512)  // catch i == 511
    {
        switch (rec_state_)
        {
        case WAIT_HEADER:
            if (c == sframe_) {
                rec_state_ = RECEIVER_STATUS::IN_MSG;   // Zeichen wird nicht gespeichert
            }
            break;

        case AFTER_ESC:
            b[i++] = c;
            rec_state_ = RECEIVER_STATUS::IN_MSG;
            break;

        case IN_MSG:
            if (c == stframe_) {
                b[i] = '\0';        // Null termination
                read_buf_.append(b);
                sgd_msgs::msg::Serial ser_msg;
                ser_msg.header.stamp = now();
                ser_msg.port = get_parameter("port").as_string();
                ser_msg.msg = read_buf_;

                pub_serial->publish(ser_msg);
                rec_state_ = raw_ ? RECEIVER_STATUS::IN_MSG : RECEIVER_STATUS::WAIT_HEADER;
                
                long t = round(now().seconds() * 1000) - time_at_start_;
                std::string ts = std::to_string(t);
                ts.append(",");
                if (log_ && file_.is_open()) {file_ << ts << read_buf_ << '\n';}
                
                read_buf_.clear();
                c = '\0';
                return;     // Warum sendet der publisher nur, wenn hier returned wird??
            } else if (c == ESC_CHAR && !raw_) {
                rec_state_ = RECEIVER_STATUS::AFTER_ESC;
            } else {
                b[i++] = c;
            }
            break;

        default:
            // Error
            break;
        }
    }

    if (num_bytes < 0) {
        RCLCPP_WARN(get_logger(), "Error reading %s", strerror(errno));
    }
    if (rec_state_ != WAIT_HEADER)
    {   // wir sind noch in der message aber es ist nichts mehr in serial
        read_buf_.append(b);
    }
}

void
Serial::write_serial(const sgd_msgs::msg::Serial::SharedPtr msg_)
{
    std::string s = msg_->msg;
    unsigned char c[512];
    std::memset(&c, '\0', sizeof(c));

    uint16_t i;
    for (i = 0; i < s.length(); i++)
    {
        c[i] = s[i];
    }
    c[i++] = '\r';
    c[i++] = '\n';
    int num_bytes = write(serial_port_, c , i);

    if (num_bytes < 0)
    {
        RCLCPP_WARN(get_logger(), "No data written to serial");
    }
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
