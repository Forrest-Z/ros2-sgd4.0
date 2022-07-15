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

    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    // ros parameters
    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    declare_parameter("odom_topic_improved", rclcpp::ParameterValue("odom/improved"));
    declare_parameter("odom_sim_topic", rclcpp::ParameterValue("sim/odom"));
    declare_parameter("battery_state_topic", rclcpp::ParameterValue("battery"));
    declare_parameter("vel_twist_topic", rclcpp::ParameterValue("sgd_move_base"));
    declare_parameter("gps_sim_topic", rclcpp::ParameterValue("gps/sim"));

    declare_parameter("compass_gain", rclcpp::ParameterValue(0.045));
    declare_parameter("gps_orientation_gain", rclcpp::ParameterValue(0.5));

    // motor parameters
    declare_parameter("wheel_separation", rclcpp::ParameterValue(0.71));
    declare_parameter("wheel_circumference", rclcpp::ParameterValue(0.68));
    declare_parameter("sim_battery_state", rclcpp::ParameterValue(37.0));

    declare_parameter("relative", rclcpp::ParameterValue(true));

    // initialize logging
    std::string log_dir_, log_sev_;
    get_parameter("log_dir", log_dir_);
    get_parameter("log_severity", log_sev_);
    std::string log_file(log_dir_ + "/" + sgd_util::create_log_file("motor"));

    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGD << "Message: rclcpp_time; millis; lin_vel; ang_vel; x; y; phi (rad); dx; dy; dphi; batt_volt";
}

Motorcontroller::~Motorcontroller() {}

CallbackReturn
Motorcontroller::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();

    //if (is_sim_) return CallbackReturn::SUCCESS;

    // Open serial port
    std::string port;
    get_parameter("port", port);

    try
    {
        RCLCPP_INFO(get_logger(), "Open serial port %s", port.c_str());
        serial.set_start_frame('{');
        serial.set_stop_frame('}');
        serial.open_port(port, 115200);
    }
    catch(const sgd_io::io_exception& e)
    {
        RCLCPP_ERROR(get_logger(), e.what());
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    wh_fcruiser = std::make_unique<WH_FCruiser>(wheel_circum_, wheel_separation_);
    
    init_pub_sub();
    odo_new.init(0,0,0);

    double g_comp, g_gps;
    get_parameter("compass_gain", g_comp);
    get_parameter("gps_orientation_gain", g_gps);

    odo_new.setCompassGain(g_comp);
    odo_new.setGpsOrientationGain(g_gps);

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    pub_odom_->on_deactivate();
    pub_battery_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    pub_odom_.reset();
    pub_battery_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    return CallbackReturn::SUCCESS;
}

void
Motorcontroller::init_parameters()
{
    get_parameter("odom_topic", odom_topic_);
    get_parameter("odom_topic_improved", odom_topic_improved_);
    get_parameter("odom_sim_topic", odom_sim_topic_);
    get_parameter("battery_state_topic", battery_state_topic_);
    get_parameter("vel_twist_topic", vel_twist_topic_);
    get_parameter("gps_sim_topic", gps_sim_topic_);
    
    get_parameter("wheel_separation", wheel_separation_);
    get_parameter("wheel_circumference", wheel_circum_);
    get_parameter("sim_battery_state", sim_battery_state_);

    get_parameter("use_sim_time", is_sim_);
    get_parameter("relative", is_relative_);
}

void
Motorcontroller::init_pub_sub()
{
    // Publisher for odometry
    RCLCPP_DEBUG(get_logger(), "Create publisher on '%s'", odom_topic_.c_str());
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, default_qos);
    pub_odom_->on_activate();

    RCLCPP_DEBUG(get_logger(), "Create publisher on '%s'", odom_topic_improved_.c_str());
    pub_odom_improved_ = this->create_publisher<sgd_msgs::msg::OdomImproved>(odom_topic_improved_, default_qos);
    pub_odom_improved_->on_activate();

    if (is_sim_)
    {
        RCLCPP_DEBUG(get_logger(), "Subscribe to '%s'", odom_sim_topic_.c_str());
        sub_odom_sim_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_sim_topic_, default_qos,
                std::bind(&Motorcontroller::on_odom_sim_received, this, std::placeholders::_1));
    }
    else
    {
        // Receive velocity command from local controller
        RCLCPP_DEBUG(get_logger(), "Subscribe to '%s'", vel_twist_topic_.c_str());
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(vel_twist_topic_, default_qos,
            std::bind(&Motorcontroller::on_cmd_vel_received, this, std::placeholders::_1));
        RCLCPP_DEBUG(get_logger(), "Subscribe to '%s'", gps_sim_topic_.c_str());
        sub_gps_ = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(gps_sim_topic_, default_qos,
            std::bind(&Motorcontroller::on_gps_received, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(100ms, std::bind(&Motorcontroller::publish_motor_cmd, this));
        timer_serial_ = this->create_wall_timer(10ms, std::bind(&Motorcontroller::read_serial, this));
    }

    timer_vol_ = this->create_wall_timer(10000ms, std::bind(&Motorcontroller::publish_battery_state, this));

    // Publish battery state
    RCLCPP_DEBUG(get_logger(), "Create publisher on '%s'", battery_state_topic_.c_str());
    pub_battery_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_state_topic_, default_qos);
    pub_battery_->on_activate();

    // Receive initial orientation from imu
    RCLCPP_DEBUG(get_logger(), "Subscribe to 'imu' topic");
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", default_qos,
                std::bind(&Motorcontroller::on_imu_received, this, std::placeholders::_1));
}

void
Motorcontroller::read_serial()
{
    if (serial.read_serial())
    {
        std::string msg = "{" + serial.get_msg() + "}";

        try
        {
            wh_fcruiser->parse_msg(msg);
        }
        catch(const nlohmann::json::exception& e)
        {
            PLOGW << "JSON parser error: " << e.what() << '\n';
            return;
        }

        odo_new.update(wh_fcruiser->get_linear_vel(), wh_fcruiser->get_angular_vel());

        // Publish odom message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        // Twist relative to child_frame_id
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        // odom.twist.twist.angular.z = wh_fcruiser->get_angular_vel();
        odom.twist.twist.angular.z = odo_new.dphi;
        odom.twist.twist.linear.x = wh_fcruiser->get_linear_vel();
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        odom.pose.pose.position.x = odo_new.x;
        odom.pose.pose.position.y = odo_new.y;
        odom.pose.pose.position.z = 0.142176;

        // Covariance -> 6x6 matrix
        for (uint8_t i = 0; i < 36; i+=7)
        {
            // set diagonal to variance
            odom.pose.covariance[i] = 0.3;
        }

        tf2::Quaternion q;
        //q.setRPY(0, 0, wh_fcruiser->get_orientation());
        q.setRPY(0, 0, odo_new.phi);
        odom.pose.pose.orientation = tf2::toMsg(q);

        PLOGD << std::to_string(now().nanoseconds()/1E6) << ";" << std::to_string(wh_fcruiser->millis()) << ";" 
              << wh_fcruiser->get_linear_vel() << ";" << wh_fcruiser->get_angular_vel() << ";"
              << odo_new.x << ";" << odo_new.y << ";" << odo_new.phi << ";" << odo_new.dx << ";"
              << odo_new.dy << ";" << odo_new.dphi << ";" << wh_fcruiser->get_batt_voltage();
        pub_odom_->publish(odom);

        sgd_msgs::msg::OdomImproved odom_impr;
        odom_impr.dx = odo_new.dx;
        odom_impr.dy = odo_new.dy;
        pub_odom_improved_->publish(odom_impr);

        return;
    }
}

void
Motorcontroller::publish_battery_state()
{
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    if (is_sim_)
    {
        msg.voltage = sim_battery_state_;
    }
    else
    {
        if (wh_fcruiser->get_batt_voltage() < 30.0)
        {
            //TODO: put this to mcu
            RCLCPP_WARN(get_logger(), "Battery voltage low!");
        }
        msg.voltage = wh_fcruiser->get_batt_voltage();
    }
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
    odo_new.updatePhi_IMU(tf2::getYaw(msg->orientation));
}

void
Motorcontroller::on_gps_received(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg)
{
    odo_new.updateLoc_GPS(msg->pose.position.x, msg->pose.position.y);
}

void
Motorcontroller::on_odom_sim_received(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (is_relative_)
    {
        // set initial pose
        initial_pos_.first = msg->pose.pose.position.x;
        initial_pos_.second = msg->pose.pose.position.y;
        is_relative_ = false;
    }

    msg->pose.pose.position.x -= initial_pos_.first;
    msg->pose.pose.position.y -= initial_pos_.second;

    pub_odom_->publish(*msg);
}

void
Motorcontroller::publish_motor_cmd()
{
    std::string msg;

    if ((now().seconds() - cmd_vel_seconds_) > 1)
    {
        msg = "0,0";
    }
    else
    {
        msg = wh_fcruiser->cmd_vel(last_cmd_vel_.linear.x, last_cmd_vel_.angular.z);
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
