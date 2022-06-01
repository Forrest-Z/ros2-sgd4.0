#include "feather_handle/feather_handle_ros.hpp"

namespace sgd_hardware_drivers
{

#define PI 3.14159265

using namespace std::chrono_literals;   // if a timer is used

Feather_Handle_ROS::Feather_Handle_ROS():
    rclcpp_lifecycle::LifecycleNode("feather_handle_ros")
{
    RCLCPP_INFO(get_logger(), "Creating");

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("imu_topic", rclcpp::ParameterValue("imu"));
    declare_parameter("imu_temp_topic", rclcpp::ParameterValue("imu/temp"));
    
    declare_parameter("cmd_vel_topic", rclcpp::ParameterValue("cmd_vel"));
    declare_parameter("sgd_move_topic", rclcpp::ParameterValue("cmd_vel_laser"));
    
    // Logging and diagnostics
    declare_parameter("log_dir", rclcpp::ParameterValue("log"));
}

Feather_Handle_ROS::~Feather_Handle_ROS()
{
    // Destroy
}

CallbackReturn
Feather_Handle_ROS::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();

    if (&log_dir_.back() != std::string("/"))    log_dir_.append("/");
    RCLCPP_DEBUG(get_logger(), "Initialize bno and laser with log dir %s", log_dir_.c_str());
    bno055_ = std::make_unique<BNO055>(log_dir_);
    vl53l1x_ = std::make_unique<VL53L1X>(log_dir_);

    // initialize serial
    std::string port_;
    get_parameter("port", port_);
    RCLCPP_INFO(get_logger(), "Open port: %s", port_.c_str());
    serial.set_start_frame('{');
    serial.set_stop_frame('\n');
    serial.open_port(port_, 115200);

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Activating");

    init_pub_sub();

    imu_pub_->on_activate();
    temp_pub_->on_activate();
    laser_pub_->on_activate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Deactivating");

    imu_pub_->on_deactivate();
    temp_pub_->on_deactivate();
    laser_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Cleanup");
    imu_pub_.reset();
    temp_pub_.reset();
    laser_pub_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
}

void
Feather_Handle_ROS::init_parameters()
{
    get_parameter("log_dir", log_dir_);
    get_parameter("imu_topic", imu_topic_);
    get_parameter("imu_temp_topic", imu_temp_topic_);

    get_parameter("sgd_move_topic", sgd_move_topic_);
    get_parameter("cmd_vel_topic", cmd_vel_topic_);
}

void
Feather_Handle_ROS::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init publisher and subscriber");

    // create timer
    timer_ = this->create_wall_timer(10ms, std::bind(&Feather_Handle_ROS::read_serial, this));

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, default_qos);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(imu_temp_topic_, default_qos);

    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos,
            std::bind(&Feather_Handle_ROS::on_cmd_vel_received, this, std::placeholders::_1));
    laser_pub_ = create_publisher<geometry_msgs::msg::Twist>(sgd_move_topic_, default_qos);
}

void
Feather_Handle_ROS::read_serial()
{
    if (serial.read_serial())
    {
        // parse msg
        std::string msg = "{" + serial.get_msg();
        auto i_bno = bno055_->parse_msg(msg);
        if (i_bno == 1)
        {
            // publish acceleration/heading/gyro/etc. data
            publish_imu();
        }

        auto i_vl = vl53l1x_->parse_msg(msg);
        if (i_vl > 0)
        {
            // publish laser range data
            cmd_laser_time_ = now().seconds();
        }
    }
}

void
Feather_Handle_ROS::publish_imu()
{
    sensor_msgs::msg::Imu msg_;
    msg_.header.stamp = now();
    msg_.header.frame_id = "imu_link";

    // orientation
    auto o = bno055_->get_euler();
    tf2::Quaternion q;
    q.setRPY(o.data[0], o.data[1], o.data[2]);
    msg_.orientation = tf2::toMsg(q);
    //RCLCPP_INFO(get_logger(), "IMU orientation: %.2f", o.data[2]);

    // acceleration
    auto a = bno055_->get_acc();
    msg_.linear_acceleration.x = a.data[0];
    msg_.linear_acceleration.y = a.data[1];
    msg_.linear_acceleration.z = a.data[2];

    // gyro
    auto g = bno055_->get_gyro();
    msg_.angular_velocity.x = g.data[0];
    msg_.angular_velocity.y = g.data[1];
    msg_.angular_velocity.z = g.data[2];

    imu_pub_->publish(msg_);
}

void
Feather_Handle_ROS::on_cmd_vel_received(geometry_msgs::msg::Twist::SharedPtr msg_)
{
    geometry_msgs::msg::Twist cmd_vel;
    if (vl53l1x_->get_vel_p() == 0)
    {
        //RCLCPP_INFO(get_logger(), "Laser vel is 0");
        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
        laser_pub_->publish(cmd_vel);
    }
    else if ((now().seconds() - cmd_laser_time_) < 1)
    {
        cmd_vel.linear.x = msg_->linear.x * vl53l1x_->get_vel_p();
        cmd_vel.angular.z = msg_->angular.z * vl53l1x_->get_vel_p();

        // RCLCPP_INFO(get_logger(), "cmd_vel (lin, ang): %.2f %.2f --> cmd_vel_laser: %.2f, %.2f",
        //             msg_->linear.x, msg_->angular.z, cmd_vel.linear.x, cmd_vel.angular.z);

        laser_pub_->publish(cmd_vel);
    }
}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Feather_Handle_ROS>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
