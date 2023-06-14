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

    declare_parameter("visualize_compass_topic", rclcpp::ParameterValue("visual/compass"));
    declare_parameter("visualize_laser_topic", rclcpp::ParameterValue("visual/vl53l1x"));

    // initialize logging
    std::string log_sev_ = get_parameter("log_severity").as_string();
    std::string log_file(get_parameter("log_dir").as_string() + "/feather_handle.csv");
    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created feather_handle node. PLOG logging severity is %s", log_sev_.c_str());
    RCLCPP_INFO(get_logger(), "Created feather_handle node. Save log file to %s", log_file.c_str());

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
    get_parameter("use_sim_time", is_sim_);

    if (is_sim_)    return CallbackReturn::SUCCESS;
    
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
    // visualization
    std::string imu_topic_ = get_parameter("imu_topic").as_string();
    std::string vis_compass_topic_ = get_parameter("visualize_compass_topic").as_string();
    if (!vis_compass_topic_.empty())
    {
        PLOGD.printf("Create publisher on topic %s", vis_compass_topic_.c_str());
        vis_compass_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(vis_compass_topic_, default_qos);
        visualize_compass = true;
        vis_compass_pub_->on_activate();

        if (is_sim_)
        {
            // create subscriber for imu data
            PLOGD.printf("Create subscription for topic %s", imu_topic_.c_str());
            sub_imu_sim_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS(rclcpp::SensorDataQoS()),
                std::bind(&Feather_Handle_ROS::publish_imu_visual, this, std::placeholders::_1));
        }
        return;
    }

    // create timer
    timer_ = this->create_wall_timer(10ms, std::bind(&Feather_Handle_ROS::read_serial, this));

    PLOGD.printf("Create publisher on topic %s", imu_topic_.c_str());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, default_qos);

    std::string imu_temp_topic_ = get_parameter("imu_temp_topic").as_string();
    PLOGD.printf("Create publisher on topic %s", imu_temp_topic_.c_str());
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(imu_temp_topic_, default_qos);

    std::string cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    PLOGD.printf("Create subscription for topic %s", cmd_vel_topic_.c_str());
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos,
            std::bind(&Feather_Handle_ROS::on_cmd_vel_received, this, std::placeholders::_1));

    std::string sgd_move_topic_ = get_parameter("sgd_move_topic").as_string();
    PLOGD.printf("Create publisher on topic %s", sgd_move_topic_.c_str());
    laser_pub_ = create_publisher<geometry_msgs::msg::Twist>(sgd_move_topic_, default_qos);

    // TODO visualization for Laser 1D

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
    auto msg_ = std::make_shared<sensor_msgs::msg::Imu>();
    // sensor_msgs::msg::Imu msg_;
    msg_->header.stamp = now();
    msg_->header.frame_id = "imu_link";

    // orientation
    auto o = bno055_->get_euler();
    tf2::Quaternion q;
    q.setRPY(o.data[0], o.data[1], o.data[2]);
    msg_->orientation = tf2::toMsg(q);

    // acceleration
    auto a = bno055_->get_acc();
    msg_->linear_acceleration.x = a.data[0];
    msg_->linear_acceleration.y = a.data[1];
    msg_->linear_acceleration.z = a.data[2];

    // gyro
    auto g = bno055_->get_gyro();
    msg_->angular_velocity.x = g.data[0];
    msg_->angular_velocity.y = g.data[1];
    msg_->angular_velocity.z = g.data[2];

    auto m = bno055_->get_magnet();
    // Covariance -> 3x3 matrix
    for (uint8_t i = 0; i < 9; i+=4)
    {
        // set diagonal to variance
        msg_->orientation_covariance[i] = 0.05;
    }

    PLOGD.printf("%u; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f", bno055_->get_millis(),
        o.data[0], o.data[1], o.data[2], g.data[0], g.data[1], g.data[2],
        m.data[0], m.data[1], m.data[2], a.data[0], a.data[1], a.data[2]);

    imu_pub_->publish(*msg_);

    if (visualize_compass)
    {
        publish_imu_visual(msg_);
    }
}

void
Feather_Handle_ROS::on_cmd_vel_received(geometry_msgs::msg::Twist::SharedPtr msg_)
{
    geometry_msgs::msg::Twist cmd_vel;
    if (vl53l1x_->get_vel_p() == 0)
    {
        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.0;
        laser_pub_->publish(cmd_vel);
    }
    else if ((now().seconds() - cmd_laser_time_) < 1)
    {
        cmd_vel.linear.x = msg_->linear.x * vl53l1x_->get_vel_p();
        cmd_vel.angular.z = msg_->angular.z * vl53l1x_->get_vel_p();
        laser_pub_->publish(cmd_vel);
    }
}

void
Feather_Handle_ROS::publish_imu_visual(sensor_msgs::msg::Imu::SharedPtr msg)
{
    // publish visualization message
    geometry_msgs::msg::PoseStamped compass_;
    compass_.header.frame_id = "imu_link";
    compass_.header.stamp = now();
    compass_.pose.orientation = msg->orientation;

    // TODO: Drehung des Robos rausrechnen, sonst ist der Winkel falsch
    
    vis_compass_pub_->publish(compass_);
}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Feather_Handle_ROS>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
