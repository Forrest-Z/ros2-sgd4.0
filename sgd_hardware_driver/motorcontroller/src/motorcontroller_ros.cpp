/*
 *  Some text
 */

# include "motorcontroller/motorcontroller_ros.hpp"

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;

Motorcontroller::Motorcontroller() : rclcpp_lifecycle::LifecycleNode("motorcontroller")
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // ros parameters
    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    //declare_parameter("msg_regex", rclcpp::ParameterValue("HS:(\\d+),Rm:(-?\\d+),Lm:(-?\\d+),V:(\\d+),T:(\\d+),S:(\\d?)"));
    declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    declare_parameter("battery_state_topic", rclcpp::ParameterValue("battery"));
    declare_parameter("vel_twist_topic", rclcpp::ParameterValue("sgd_move_base"));

    // motor parameters
    declare_parameter("wheel_separation", rclcpp::ParameterValue(0.71));
    declare_parameter("wheel_circumference", rclcpp::ParameterValue(0.68));
    declare_parameter("kp", rclcpp::ParameterValue(0.2));
}

Motorcontroller::~Motorcontroller()
{

}

CallbackReturn
Motorcontroller::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();

    // Open serial port
    std::string port;
    get_parameter("port", port);

    RCLCPP_DEBUG(get_logger(), "Open serial port %s", port.c_str());
    serial.set_start_frame('{');
    serial.set_stop_frame('\n');
    serial.open_port(port, 115200);

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    wh_fcruiser = std::make_unique<WH_FCruiser>(wheel_circum_, wheel_separation_);

    init_pub_sub();

    pub_odom_->on_activate();
    pub_battery_->on_activate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    pub_odom_->on_deactivate();
    pub_battery_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    pub_odom_.reset();
    pub_battery_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
}

void
Motorcontroller::init_parameters()
{
    get_parameter("port", port_);
    //get_parameter("msg_regex", msg_regex_);
    get_parameter("odom_topic", odom_topic_);
    RCLCPP_INFO(get_logger(), "oodom topic is %s", odom_topic_.c_str());
    get_parameter("battery_state_topic", battery_state_topic_);
    get_parameter("vel_twist_topic", vel_twist_topic_);
    
    get_parameter("wheel_separation", wheel_separation_);
    get_parameter("wheel_circumference", wheel_circum_);
}

void
Motorcontroller::init_pub_sub()
{
    //regex_ = std::regex(msg_regex_);

    timer_ = this->create_wall_timer(100ms, std::bind(&Motorcontroller::publish_motor_cmd, this));
    timer_serial_ = this->create_wall_timer(10ms, std::bind(&Motorcontroller::read_serial, this));
    timer_vol_ = this->create_wall_timer(10000ms, std::bind(&Motorcontroller::publish_battery_state, this));

    // Publish odometry
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, default_qos);

    // Receive velocity command from local controller
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(vel_twist_topic_, default_qos,
        std::bind(&Motorcontroller::on_cmd_vel_received, this, std::placeholders::_1));

    // Receive initial orientation from imu
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", default_qos,
                std::bind(&Motorcontroller::on_imu_received, this, std::placeholders::_1));

    // Publish battery state
    pub_battery_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_state_topic_, default_qos);
}

void
Motorcontroller::read_serial()
{
    if (serial.read_serial() && imu_msgs_rec_ > 10)
    {
        std::string msg = "{" + serial.get_msg();
        wh_fcruiser->parse_msg(msg);

        // Publish odom message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        // Twist relative to child_frame_id
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = wh_fcruiser->get_angular_vel();
        odom.twist.twist.linear.x = wh_fcruiser->get_linear_vel();
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        wh_fcruiser->get_position(odom.pose.pose.position.x, odom.pose.pose.position.y);
        odom.pose.pose.position.z = 0.142176;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, wh_fcruiser->get_orientation() + initial_orientation_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        pub_odom_->publish(odom);
    }
}

void
Motorcontroller::publish_battery_state()
{
    RCLCPP_INFO(get_logger(), "ODOM initial: %.2f, rel: %.2f", initial_orientation_, wh_fcruiser->get_orientation());

    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";

    if (wh_fcruiser->get_batt_voltage() < 30.0)
    {
        RCLCPP_WARN(get_logger(), "Battery voltage low!");
    }

    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.voltage = wh_fcruiser->get_batt_voltage();
    pub_battery_->publish(msg);
}

void
Motorcontroller::on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_seconds_ = now().seconds();
    last_cmd_vel_ = *msg;
}

void
Motorcontroller::on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // take 10 measurements and calculate initial rotation
    if (imu_msgs_rec_ < 10)
    {
        tmp_init_ori_ += tf2::getYaw(msg->orientation);
        imu_msgs_rec_++;
    } else if (imu_msgs_rec_ == 10)
    {
        initial_orientation_ = tmp_init_ori_ / 10.0;
        imu_msgs_rec_++;
    }
}

void
Motorcontroller::publish_motor_cmd()
{
    std::string msg;

    if ((now().seconds() - cmd_vel_seconds_) > 5)
    {
        msg = "0,0";
    }
    else
    {
        msg = wh_fcruiser->cmd_vel(last_cmd_vel_.linear.x, last_cmd_vel_.angular.z);
        //msg = std::to_string((int)round(last_cmd_vel_.linear.x * 250)) + ","
        //      + std::to_string((int)round(last_cmd_vel_.angular.z * 250));
    }
    
    serial.write_serial(msg);
}

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Motorcontroller>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
