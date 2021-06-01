#include "gps/navilock_ublox6_gps.hpp"

namespace sgd_sensors
{

using namespace std::chrono_literals;   // if a timer is used

Navilock_UBlox6_GPS::Navilock_UBlox6_GPS():
    nav2_util::LifecycleNode("navilock_ublox6_gps", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("xml_file", rclcpp::ParameterValue("/home/ipp/dev_ws/src/ros2-sgd4.0/sensors/gps/data/nmea.xml"));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{ 
    // Destroy
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();
    RCLCPP_INFO(get_logger(), "Xml-File: %s", xml_file_.c_str());
    nmea_parser_ = std::shared_ptr<Nmea_Parser>(new Nmea_Parser(xml_file_));

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");
    publisher_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publisher_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Navilock_UBlox6_GPS::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("xml_file", xml_file_);
}

void
Navilock_UBlox6_GPS::init_pub_sub()
{
    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", default_qos);
    subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(
        serial_topic, default_qos, std::bind(&Navilock_UBlox6_GPS::read_msg, this, std::placeholders::_1));

    gps_counter_ = 0;
    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on topic %s.",
            'gps', serial_topic.c_str());
}

void
Navilock_UBlox6_GPS::read_msg(const sgd_msgs::msg::Serial::SharedPtr msg)
{
    std::string line = msg->msg;
    nmea_parser_->parse_line(line);

    if (nmea_parser_->msg_complete())
    {
        if (gps_counter_ > 2)
        {
            // filter
            // publish message, reset counter
            gps_counter_ = 0;
        }
        else
        {
            
            gps_counter_++;
        }
        



        


        // Alle Daten sind da und können gepublished werden.
        sensor_msgs::msg::NavSatFix nsf;
        nsf.latitude = nmea_parser_->latitude();
        nsf.longitude = nmea_parser_->longitude();
        nsf.header.stamp.sec = nmea_parser_->time();
        nsf.header.stamp.nanosec = (nmea_parser_->time() - floor(nmea_parser_->time())) * 1E9;

        nsf.status.status = nmea_parser_->fix() > 1 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                    : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

        RCLCPP_INFO(get_logger(), "GPS lat: %.7f, lon %.7f", nmea_parser_->latitude(), nmea_parser_->longitude());

        publisher_->publish(nsf);

        // Clear old message 
        nmea_parser_->clear();
    }

}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_sensors::Navilock_UBlox6_GPS>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
