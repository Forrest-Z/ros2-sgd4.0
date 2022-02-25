/*
 *  Some text
 */

# include "sgd_util/logger.hpp"

namespace sgd_util
{

Logger::Logger() : rclcpp::Node("logger")
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    // Add parameters

    imu_topic_ = declare_parameter<std::string>("imu_topic", "scan");
    gps_topic_ = declare_parameter<std::string>("gps_topic", "tf");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    output_folder_ = declare_parameter<std::string>("output_folder", "log");

    time_at_start_ = round(now().nanoseconds() / 1.0E6);
    std::string time = std::to_string(time_at_start_); // time in millis

    scan_filename_ = output_folder_ + "/scan_" + time + ".log";
    //out_imu_.open(scan_filename_, std::ios::out | std::ios::trunc);
    //out_gps_.open(output_folder_ + "/gps_" + time + ".log", std::ios::out | std::ios::trunc);
    //out_odom_.open(output_folder_ + "/odom_" + time + ".log", std::ios::out | std::ios::trunc);

    // Initialize parameters, pub/sub, services, etc.
    init_pub_sub();
    init_transforms();
}

Logger::~Logger()
{
    // Destroy
}

void
Logger::init_pub_sub()
{
    //sub_gps_ = this->create_subscription<tf2_msgs::msg::TFMessage>(gps_topic_, default_qos,
    //    std::bind(&Logger::gps_callback, this, std::placeholders::_1));

    // Publish odometry
    sub_imu_ = this->create_subscription<sensor_msgs::msg::LaserScan>(imu_topic_, default_qos,
        std::bind(&Logger::imu_callback, this, std::placeholders::_1));

    // Receive velocity command from local controller
    //sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, default_qos,
    //    std::bind(&Logger::odom_callback, this, std::placeholders::_1));
}

void
Logger::init_transforms()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void
Logger::imu_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_)
{
    // linear acceleration, angular velocity, orientation
    out_imu_.open(scan_filename_, std::ios::out | std::ios::app);
    out_imu_ << "Laser: ";
    out_imu_ << time_to_string();
    out_imu_ << stamp_to_string(msg_->header);

    out_imu_ << "\tTransform: ";

    try
    {
        // transformation from map -> base_link in local cooridinates
        geometry_msgs::msg::TransformStamped tf_ = tf_buffer_->lookupTransform("map", "base_link",
                    rclcpp::Time(0), rclcpp::Duration(5,0));
        out_imu_ << time_to_string();
        out_imu_ << stamp_to_string(tf_.header);
    } catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    out_imu_ << "\n";
    out_imu_.close();
}

std::string
Logger::time_to_string()
{
  double t = now().nanoseconds() / 1.0E9; // - time_at_start_;
  std::string ts = to_string(t);
  ts.append(",");
  return ts;
}

std::string
Logger::vec3_to_string(geometry_msgs::msg::Vector3 vec3)
{
  std::string s;
  s = std::to_string(vec3.x) + "," + std::to_string(vec3.y) + "," + std::to_string(vec3.z);
  return s;
}

std::string
Logger::stamp_to_string(std_msgs::msg::Header header)
{
    double t = header.stamp.sec + header.stamp.nanosec / 1.0E9;
    return to_string(t);
}

std::string
Logger::to_string(double time_s, std::string format)
{
    int sz = std::snprintf(nullptr, 0, format.c_str(), time_s);
    char buf[sz + 1]; // +1 for null terminator
    std::snprintf(&buf[0], sz+1, format.c_str(), time_s);
    return std::string(buf);
}

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_util::Logger>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
