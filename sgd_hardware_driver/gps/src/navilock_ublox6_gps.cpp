#include "gps/navilock_ublox6_gps.hpp"

namespace sgd_hardware
{

using namespace std::chrono_literals;   // if a timer is used

Navilock_UBlox6_GPS::Navilock_UBlox6_GPS():
    nav2_util::LifecycleNode("navilock_ublox6_gps", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("xml_file", rclcpp::ParameterValue("/home/ipp/dev_ws/src/ros2-sgd4.0/sensors/gps/data/nmea.xml"));
    add_parameter("output_folder", rclcpp::ParameterValue("log"));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{
    // Destroy
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");
    time_at_start_ = round(now().nanoseconds() / 1.0E6);
    std::string time = std::to_string(time_at_start_); // time in millis

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();
    init_transforms();
    nmea_parser_ = std::shared_ptr<Nmea_Parser>(new Nmea_Parser(xml_file_));

    out_gps_.open(output_folder_ + "/gps_" + time + ".log", std::ios::out | std::ios::trunc);

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
    out_gps_.close();

    return nav2_util::CallbackReturn::SUCCESS;
}

void
Navilock_UBlox6_GPS::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("xml_file", xml_file_);
    get_parameter("output_folder", output_folder_);
}

void
Navilock_UBlox6_GPS::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init pub sub");
    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", default_qos);
    subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(
        serial_topic, default_qos, std::bind(&Navilock_UBlox6_GPS::read_msg, this, std::placeholders::_1));

    gps_counter_ = 0;
    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on topic %s.",
            "gps", serial_topic.c_str());
}

void
Navilock_UBlox6_GPS::init_transforms()
{
    RCLCPP_DEBUG(get_logger(), "Init transforms");
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
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
        double hdop = nmea_parser_->get_data<double>("hdop");

        nsf.status.status = (nmea_parser_->fix() > 1 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                    : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);

        // save data to file
        out_gps_ << time_to_string();
        out_gps_ << std::to_string(nsf.latitude) << ",";
        out_gps_ << std::to_string(nsf.longitude) << ",";
        out_gps_ << std::to_string(hdop) << "\n";

        //RCLCPP_INFO(get_logger(), "GPS lat: %.7f, lon %.7f", nmea_parser_->latitude(), nmea_parser_->longitude());

        // publish message and transforms
        publisher_->publish(nsf);

        geometry_msgs::msg::TransformStamped gps_tf;
        gps_tf.header.stamp = now();
        gps_tf.header.frame_id = "map";
        gps_tf.child_frame_id = "gps";

        auto local_pos = sgd_util::WGS84_to_local(nmea_parser_->latitude(), nmea_parser_->longitude());
        xy lpos;
        lpos.x = local_pos.first;
        lpos.y = local_pos.second;

        gps_tf.transform.translation.x = lpos.x;
        gps_tf.transform.translation.y = lpos.y;
        gps_tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        gps_tf.transform.rotation = tf2::toMsg(q);
        
        tf_broadcaster_->sendTransform(gps_tf);

        // Clear old message 
        nmea_parser_->clear();
    }
}

double
Navilock_UBlox6_GPS::get_direction_from_previous()
{
    return 0.0;
}

std::string
Navilock_UBlox6_GPS::time_to_string()
{
  long t = round(now().nanoseconds() / 1.0E6);
  std::string ts = std::to_string(t);
  ts.append(",");
  return ts;
}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware::Navilock_UBlox6_GPS>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
