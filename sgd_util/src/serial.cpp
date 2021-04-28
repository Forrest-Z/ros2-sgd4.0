#include "sgd_util/serial.hpp"

namespace sgd_util
{

using namespace std::chrono_literals;   // if a timer is used

Serial::Serial():
    nav2_util::LifecycleNode("example_node", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("baud_rate", rclcpp::ParameterValue(9600));
    add_parameter("read_write", rclcpp::ParameterValue("rw"));
}

Serial::~Serial()
{
    // Destroy
}

nav2_util::CallbackReturn
Serial::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    init_serial(port_.c_str(), baud_rate_);

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Serial::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    pub_serial->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Serial::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    pub_serial->on_deactivate();

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
    get_parameter("baud_rate", baud_rate_);
    get_parameter("read_write", read_write_);
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

int
Serial::init_serial(const char *port, const int baud)
{
    struct termios tty_;

    serial_port_ = open(port, O_RDWR);

    if (serial_port_ < 0)
    {
        RCLCPP_ERROR(get_logger(), "Error %i from open: %s", errno, strerror(errno));
        return 1;
    }

    if (tcgetattr(serial_port_, &tty_) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
        RCLCPP_ERROR(get_logger(), "Check all connections and try again.");
        return 1;       // TODO: Error handling
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
        return 1;
    }
 
    return 0;
}

void
Serial::read_serial()
{
    // read serial port
    char b [512];
    std::memset(&b, '\0', sizeof(b));

    // read one char per read
    int num_bytes;
    char c;
    int i = 0;
    while ((num_bytes = read(serial_port_, &c, 1)) > 0)
    {
        b[i] = c;
        i++;

        if (c == '\n')
        {
            read_buf_.append(b);
            sgd_msgs::msg::Serial ser_msg;
            ser_msg.header.stamp = now();
            ser_msg.port = get_parameter("port").as_string();
            ser_msg.msg = read_buf_;

            pub_serial->publish(ser_msg);
            read_buf_.clear();
            c = '\0';
            return;
        }

    }

    if (num_bytes < 0) {
        RCLCPP_WARN(get_logger(), "Error reading %s", strerror(errno));
        return;
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



}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_util::Serial>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
