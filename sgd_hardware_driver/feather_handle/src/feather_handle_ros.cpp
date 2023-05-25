#include "feather_handle/feather_handle_ros.hpp"

namespace sgd_hardware_drivers
{
using namespace std::chrono_literals;   // if a timer is used

Feather_Handle_ROS::Feather_Handle_ROS():
    rclcpp_lifecycle::LifecycleNode("feather_handle_ros")
{
    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("imu_topic", rclcpp::ParameterValue("imu"));
    declare_parameter("imu_temp_topic", rclcpp::ParameterValue("imu/temp"));
    
    declare_parameter("cmd_vel_topic", rclcpp::ParameterValue("cmd_vel"));
    declare_parameter("sgd_move_topic", rclcpp::ParameterValue("cmd_vel_laser"));

    // initialize logging
    std::string log_sev_ = get_parameter("log_severity").as_string();
    std::string log_file(get_parameter("log_dir").as_string() + "/feather_handle.csv");
    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created feather_handle node. PLOG logging severity is %s", log_sev_.c_str());
    RCLCPP_INFO(get_logger(), "Created feather_handle node. Savo log file to %s", log_file.c_str());

    PLOGD << "millis; roll; pitch; yaw; gyro (x3);;; magneto (x3);;; accel (x3)";
}

Feather_Handle_ROS::~Feather_Handle_ROS()
{
    // Destroy
}

CallbackReturn
Feather_Handle_ROS::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Configuring...";

    bno055_ = std::make_unique<BNO055>();
    vl53l1x_ = std::make_unique<VL53L1X>();

    // initialize serial
    std::string port_ = get_parameter("port").as_string();
    try
    {
        PLOGI.printf("Open port: %s", port_.c_str());
        serial.set_start_frame('{');
        serial.set_stop_frame('\n');
        serial.open_port(port_, 115200);
    }
    catch(const sgd_io::io_exception& e)
    {
        RCLCPP_ERROR(get_logger(), e.what());
        return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Activating...";
    init_pub_sub();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Deactivating...";
    imu_pub_->on_deactivate();
    temp_pub_->on_deactivate();
    laser_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Cleaning up...";
    imu_pub_.reset();
    temp_pub_.reset();
    laser_pub_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Feather_Handle_ROS::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Shutting down...";
    return CallbackReturn::SUCCESS;
}

void
Feather_Handle_ROS::init_pub_sub()
{
    // create timer
    timer_ = this->create_wall_timer(10ms, std::bind(&Feather_Handle_ROS::read_serial, this));

    std::string imu_topic_ = get_parameter("imu_topic").as_string();
    RCLCPP_DEBUG(get_logger(), "Create publisher on topic %s", imu_topic_.c_str());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, default_qos);

    std::string imu_temp_topic_ = get_parameter("imu_temp_topic").as_string();
    RCLCPP_DEBUG(get_logger(), "Create publisher on topic %s", imu_temp_topic_.c_str());
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(imu_temp_topic_, default_qos);

    std::string cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    RCLCPP_DEBUG(get_logger(), "Create subscription for topic %s", cmd_vel_topic_.c_str());
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos,
            std::bind(&Feather_Handle_ROS::on_cmd_vel_received, this, std::placeholders::_1));

    std::string sgd_move_topic_ = get_parameter("sgd_move_topic").as_string();
    RCLCPP_DEBUG(get_logger(), "Create publisher on topic %s", sgd_move_topic_.c_str());
    laser_pub_ = create_publisher<geometry_msgs::msg::Twist>(sgd_move_topic_, default_qos);

    imu_pub_->on_activate();
    temp_pub_->on_activate();
    laser_pub_->on_activate();
}

void
Feather_Handle_ROS::read_serial()
{
    if (serial.read_serial())
    {
        // parse msg
        std::string msg = "{" + serial.get_msg();
        PLOGV << "json: " << msg;

        try
        {
            auto i_bno = bno055_->parse_msg(msg);
            if (i_bno)
            {
                // publish acceleration/heading/gyro/etc. data
                publish_imu();
            }

            auto i_vl = vl53l1x_->parse_msg(msg);
            if (i_vl)
            {
                // publish laser range data
                cmd_laser_time_ = now().seconds();
            }
        }
        catch(const std::exception& e)
        {
            PLOGE.printf("Error parsing json: %s in json: %s", e.what(), msg.c_str());
            RCLCPP_ERROR(get_logger(), "Error parsing json: %s in json: %s", e.what(), msg.c_str());
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

    auto m = bno055_->get_magnet();
    // Covariance -> 3x3 matrix
    for (uint8_t i = 0; i < 9; i+=4)
    {
        // set diagonal to variance
        msg_.orientation_covariance[i] = 0.05;
    }

    PLOGD << bno055_->get_millis() << ";" 
          << std::to_string(o.data[0]) << ";" << std::to_string(o.data[1]) << ";" << std::to_string(o.data[2]) << ";"
          << std::to_string(g.data[0]) << ";" << std::to_string(g.data[1]) << ";" << std::to_string(g.data[2]) << ";"
          << std::to_string(m.data[0]) << ";" << std::to_string(m.data[1]) << ";" << std::to_string(m.data[2]) << ";"
          << std::to_string(a.data[0]) << ";" << std::to_string(a.data[1]) << ";" << std::to_string(a.data[2]);

    imu_pub_->publish(msg_);
}

void
Feather_Handle_ROS::on_cmd_vel_received(geometry_msgs::msg::Twist::SharedPtr msg_)
{
    geometry_msgs::msg::Twist cmd_vel;
    //PLOGD << "Received velocity command (vel_x, rot_z) " << cmd_vel.linear.x << ", " << cmd_vel.angular.z;
    if (vl53l1x_->get_vel_p() == 0)
    {
        //RCLCPP_INFO(get_logger(), "Laser vel is 0");
        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
        //PLOGD << "Publish velocity command (vel_x, rot_z) " << cmd_vel.linear.x << ", " << cmd_vel.angular.z;
        laser_pub_->publish(cmd_vel);
    }
    else if ((now().seconds() - cmd_laser_time_) < 1)
    {
        cmd_vel.linear.x = msg_->linear.x * vl53l1x_->get_vel_p();
        cmd_vel.angular.z = msg_->angular.z * vl53l1x_->get_vel_p();
        //PLOGD << "Publish velocity command (vel_x, rot_z) " << cmd_vel.linear.x << ", " << cmd_vel.angular.z;
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
