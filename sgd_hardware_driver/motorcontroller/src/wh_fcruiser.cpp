/*
 *  Some text
 */

# include "motorcontroller/wh_fcruiser.hpp"

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;

WH_Fcruiser::WH_Fcruiser() : nav2_util::LifecycleNode("wh_fcruiser", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    // Add parameters
    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("msg_regex", rclcpp::ParameterValue("HS:(\\d+),Rm:(-?\\d+),Lm:(-?\\d+),V:(\\d+),T:(\\d+),S:(\\d?)"));
    add_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    add_parameter("battery_state_topic", rclcpp::ParameterValue("battery"));
    add_parameter("vel_twist_topic", rclcpp::ParameterValue("sgd_move_base"));
    add_parameter("wheel_separation", rclcpp::ParameterValue(0.71));
    add_parameter("wheel_circumference", rclcpp::ParameterValue(0.68));
    add_parameter("kp", rclcpp::ParameterValue(0.2));

    // Logging and diagnostics
}

WH_Fcruiser::~WH_Fcruiser()
{
    // Destroy
    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    msg.msg = "0,0";
    pub_motor_->publish(msg);
}

nav2_util::CallbackReturn
WH_Fcruiser::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    pub_odom_->on_activate();
    pub_motor_->on_activate();
    pub_battery_->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    pub_odom_->on_deactivate();
    pub_motor_->on_deactivate();
    pub_battery_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    sgd_msgs::msg::Serial msg;
    msg.header.stamp = now();
    msg.msg = "0,0";
    pub_motor_->publish(msg);

    pub_odom_.reset();
    pub_motor_.reset();
    pub_battery_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WH_Fcruiser::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
WH_Fcruiser::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("msg_regex", msg_regex_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("battery_state_topic", battery_state_topic_);
    get_parameter("vel_twist_topic", vel_twist_topic_);
    get_parameter("wheel_separation", wheel_separation_);
    get_parameter("wheel_circumference", wheel_circum_);
    get_parameter("kp", kp_);
}

void
WH_Fcruiser::init_pub_sub()
{
    regex_ = std::regex(msg_regex_);
    std::string topic_pub = "write_" + port_.substr(port_.find_last_of("/")+1);
    std::string topic_sub = "serial_" + port_.substr(port_.find_last_of("/")+1);

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on topic %s.",
            topic_pub.c_str(), topic_sub.c_str());

    timer_ = this->create_wall_timer(100ms, std::bind(&WH_Fcruiser::publish_motor_cmd, this));
    timer_vol_ = this->create_wall_timer(10000ms, std::bind(&WH_Fcruiser::publish_battery_state, this));

    // Communication with motorcontroller
    pub_motor_ = this->create_publisher<sgd_msgs::msg::Serial>(topic_pub,default_qos);
    sub_motor_ = this->create_subscription<sgd_msgs::msg::Serial>(topic_sub, default_qos,
        std::bind(&WH_Fcruiser::on_motor_received, this, std::placeholders::_1));

    // Publish odometry
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, default_qos);

    // Receive velocity command from local controller
    sub_data_ = this->create_subscription<geometry_msgs::msg::Twist>(vel_twist_topic_, default_qos,
        std::bind(&WH_Fcruiser::on_cmd_vel_received, this, std::placeholders::_1));

    // Publish battery state
    pub_battery_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_state_topic_, default_qos);
}

void
WH_Fcruiser::publish_battery_state()
{
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";

    if (batt_volt_ < 35)
    {
        RCLCPP_WARN(get_logger(), "Battery voltage low!");
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.voltage = batt_volt_;
    pub_battery_->publish(msg);
}

void
WH_Fcruiser::on_motor_received(const sgd_msgs::msg::Serial::SharedPtr msg)
{
    // Parse motor data
    if (msg->msg.find("HS") == std::string::npos) {return;}     // message is not from motor

    std::smatch matches;
    std::regex_search(msg->msg, matches, regex_);
    
    double meas_r_ = 0.0;   // measurement from right wheel
    double meas_l_ = 0.0;   // measurement from left wheel

    int v;
    if (matches.size() > 4) // ist gut
    {
        //time = std::stoi(matches[1]);       // in millis
        meas_r_ = std::stod(matches[2]);
        meas_l_ = std::stod(matches[3]);
        v = std::stoi(matches[4]);
    } else { return; }

    meas_r_ = meas_r_ / RIGHT_WHEEL_FACTOR;
    meas_l_ = meas_l_ / LEFT_WHEEL_FACTOR;

    rec_speed_ = ((meas_r_ + meas_l_) * wheel_circum_) / 2;
    rec_steer_ = ((meas_r_ - meas_l_) * wheel_circum_) / wheel_separation_;

    // Calculate pose
    rclcpp::Duration delta_t(now() - last_odom_msg_.header.stamp);

    pose_orie_z += rec_steer_ * delta_t.seconds();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    // Twist relative to child_frame_id
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = rec_steer_;
    odom.twist.twist.linear.x = rec_speed_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.pose.pose.position.x = last_odom_msg_.pose.pose.position.x + last_odom_msg_.twist.twist.linear.x * delta_t.seconds() * cos(pose_orie_z);
    odom.pose.pose.position.y = last_odom_msg_.pose.pose.position.y + last_odom_msg_.twist.twist.linear.x * delta_t.seconds() * sin(pose_orie_z);
    odom.pose.pose.position.z = 0.142176;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose_orie_z);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // TODO: covariance

    pub_odom_->publish(odom);
    last_odom_msg_ = odom;

    batt_volt_ = (2 * batt_volt_ + v / 100.0) / 3;
}

void
WH_Fcruiser::on_cmd_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    //double speed = msg->linear.x * 150;
    //double steer = msg->angular.z * 150;
    cmd_vel_seconds_ = now().seconds();
    last_cmd_vel_ = *msg;
}

void
WH_Fcruiser::publish_motor_cmd()
{
    sgd_msgs::msg::Serial serial;
    serial.header.stamp = now();

    if ((now().seconds() - cmd_vel_seconds_) > 5)
    {
        serial.msg = "0,0";
    }
    else
    {
        // Difference for controller
        double diff_speed_ = last_cmd_vel_.linear.x - rec_speed_;
        double diff_steer_ = last_cmd_vel_.angular.z - rec_steer_;

        last_speed_ = last_speed_ + diff_speed_ * kp_;
        last_steer_ = last_steer_ + diff_steer_ * kp_;

        RCLCPP_INFO(get_logger(), "Soll: %f, Ist: %f, Set new: %f", last_cmd_vel_.linear.x, rec_speed_, last_speed_);

        if (last_cmd_vel_.linear.x == 0 && rec_speed_ == 0)     last_speed_ = 0;
        if (last_cmd_vel_.angular.z == 0 && rec_steer_ == 0)    last_steer_ = 0;
        serial.msg = std::to_string((int)round(last_speed_ * 150)) + "," + std::to_string((int)round(last_steer_ * 150));
    }
    pub_motor_->publish(serial);
}

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::WH_Fcruiser>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
