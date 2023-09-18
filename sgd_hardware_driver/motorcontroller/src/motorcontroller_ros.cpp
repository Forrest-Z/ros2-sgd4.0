// Copyright 2023 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "motorcontroller/motorcontroller_ros.hpp"

namespace sgd_hardware_drivers
{

using namespace std::chrono_literals;

Motorcontroller::Motorcontroller() : rclcpp_lifecycle::LifecycleNode("motorcontroller")
{
    // declare logging parameters
    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    // subscription topics
    declare_parameter("gps_topic", rclcpp::ParameterValue("gps/local"));
    declare_parameter("imu_topic", rclcpp::ParameterValue("imu"));
    declare_parameter("pico_base_sub_topic", rclcpp::ParameterValue("pico_base_odom"));
    declare_parameter("odom_sim_topic", rclcpp::ParameterValue("odom/sim"));
    declare_parameter("sgd_move_topic", rclcpp::ParameterValue("sgd_move_base"));
    // publisher topics
    declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    declare_parameter("pico_base_pub_topic", rclcpp::ParameterValue("pico_base_motor"));
    declare_parameter("battery_state_topic", rclcpp::ParameterValue("battery"));

    declare_parameter("compass_gain", rclcpp::ParameterValue(0.045));
    declare_parameter("gps_orientation_gain", rclcpp::ParameterValue(0.5));

    // motor parameters
    declare_parameter("wheel_separation", rclcpp::ParameterValue(0.71));
    declare_parameter("wheel_circumference", rclcpp::ParameterValue(0.68));
    declare_parameter("sim_battery_volt", rclcpp::ParameterValue(37.0));

    declare_parameter("relative", rclcpp::ParameterValue(true));

    // initialize logging
    std::string log_dir_, log_sev_;
    get_parameter("log_dir", log_dir_);
    get_parameter("log_severity", log_sev_);
    std::string log_file(log_dir_ + "/odom.csv");
    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created motorcontroller node. PLOG logging severity is %s", log_sev_.c_str());
    RCLCPP_INFO(get_logger(), "Created motorcontroller node. Save log file to %s", log_file.c_str());
}

Motorcontroller::~Motorcontroller() {}

CallbackReturn
Motorcontroller::on_configure(const rclcpp_lifecycle::State &state __attribute__((unused)))
{
    PLOGD << "Configuring...";
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_activate(const rclcpp_lifecycle::State &state __attribute__((unused)))
{
    PLOGD << "Activating...";
    
    if (!is_sim_)
    {
        tmcm1638 = std::make_unique<TMCM1638>(get_parameter("wheel_circumference").as_double(),
                                            get_parameter("wheel_separation").as_double());
    }

    init_pub_sub();

    odo_new.init(0, 0, 0);
    odo_new.setCompassGain(get_parameter("compass_gain").as_double());
    odo_new.setGpsOrientationGain(get_parameter("gps_orientation_gain").as_double());

    PLOGD_IF(is_sim_) << "rclcpp_time; twist.linear.x; twist.angular.z; position.x; position.y; odo_new.x; odo_new.y";
    PLOGD_IF(!is_sim_) << "rclcpp_time; meas_L; meas_R; twist.linear.x; twist.angular.z; odo_new.x; odo_new.y; battery voltage";

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_deactivate(const rclcpp_lifecycle::State &state __attribute__((unused)))
{
    PLOGD << "Deactivating...";
    pub_odom_->on_deactivate();
    pub_battery_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_cleanup(const rclcpp_lifecycle::State &state __attribute__((unused)))
{
    PLOGD << "Cleaning up...";
    pub_odom_.reset();
    pub_battery_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Motorcontroller::on_shutdown(const rclcpp_lifecycle::State &state __attribute__((unused)))
{
    PLOGD << "Shutting down...";
    return CallbackReturn::SUCCESS;
}

void
Motorcontroller::init_parameters()
{
    get_parameter("sim_battery_volt", sim_battery_voltage_);

    get_parameter("use_sim_time", is_sim_);
    get_parameter("relative", is_relative_);
}

void
Motorcontroller::init_pub_sub()
{
    PLOGD << "Initialize publisher and subscriber";
    // Publisher for odometry
    std::string odom_topic_ = get_parameter("odom_topic").as_string();
    PLOGI.printf("Create publisher on topic '%s'", odom_topic_.c_str());
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, default_qos);
    pub_odom_->on_activate();

    // Publish battery state
    std::string battery_state_topic_ = get_parameter("battery_state_topic").as_string();
    PLOGI.printf("Create publisher on topic '%s'", battery_state_topic_.c_str());
    pub_battery_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_state_topic_, default_qos);
    pub_battery_->on_activate();

    std::string gps_topic_ = get_parameter("gps_topic").as_string();
    PLOGI.printf("Subscribe to '%s'", gps_topic_.c_str());
    sub_gps_ = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(gps_topic_, default_qos,
                                        std::bind(&Motorcontroller::on_gps_received, this, std::placeholders::_1));

    // Receive initial orientation from imu
    std::string imu_topic_ = get_parameter("imu_topic").as_string();
    PLOGI.printf("Subscribe to '%s'", imu_topic_.c_str());
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, default_qos,
                                        std::bind(&Motorcontroller::on_imu_received, this, std::placeholders::_1));

    if (is_sim_)
    {
        std::string odom_sim_topic_ = get_parameter("odom_sim_topic").as_string();
        PLOGI.printf("Subscribe to '%s'", odom_sim_topic_.c_str());
        sub_odom_sim_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_sim_topic_, default_qos,
                                        std::bind(&Motorcontroller::on_odom_sim_received, this, std::placeholders::_1));
    }
    else
    {
        // receive motor data via microROS
        std::string pico_base_sub_topic_ = get_parameter("pico_base_sub_topic").as_string();
        PLOGI.printf("Subscribe to '%s'", pico_base_sub_topic_.c_str());
        sub_motor_ = this->create_subscription<geometry_msgs::msg::Twist>(pico_base_sub_topic_, default_qos,
                                        std::bind(&Motorcontroller::on_motor_received, this, std::placeholders::_1));
    //}
        // publish motor commands via microROS
        std::string pico_base_pub_topic_ = get_parameter("pico_base_pub_topic").as_string();
        PLOGI.printf("Create publisher on topic '%s'", pico_base_pub_topic_.c_str());
        pub_motor_ = this->create_publisher<geometry_msgs::msg::Quaternion>(pico_base_pub_topic_, default_qos);
        pub_motor_->on_activate();

        // Receive velocity command from local controller
        std::string sgd_move_topic_ = get_parameter("sgd_move_topic").as_string();
        PLOGI.printf("Subscribe to '%s'", sgd_move_topic_.c_str());
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(sgd_move_topic_, default_qos,
                                        std::bind(&Motorcontroller::on_sgd_move_received, this, std::placeholders::_1));
        
        // init timer
        timer_ = this->create_wall_timer(100ms, std::bind(&Motorcontroller::publish_motor_cmd, this));
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
        msg.voltage = sim_battery_voltage_;
    }
    else
    {
        if (tmcm1638->get_batt_voltage() < 30.0)
        {
            // TODO: put this to mcu
            RCLCPP_WARN(get_logger(), "Battery voltage low!");
        }
        msg.voltage = tmcm1638->get_batt_voltage();
    }
    pub_battery_->publish(msg);
}

void
Motorcontroller::on_sgd_move_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_seconds_ = now().seconds();
    last_cmd_vel_ = *msg;

    // TODO: publish motor message
    // auto m = tmcm1638->cmd_vel(msg->linear.x, msg->angular.z);
    // PLOGD.printf("%.3f; %.3f; r: %.3f; l: %.3f", msg->linear.x, msg->angular.x, m.y, m.x);
    // pub_motor_->publish(m);
}

void
Motorcontroller::on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double yaw_ = tf2::getYaw(msg->orientation);
    PLOGV.printf("%.3f", yaw_);
    odo_new.updatePhi_IMU(yaw_);
}

void
Motorcontroller::on_gps_received(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg)
{
    PLOGV.printf("%.3f; %.3f", msg->pose.position.x, msg->pose.position.y);
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
        PLOGD.printf("Set initial pose to %.3f, %.3f", initial_pos_.first, initial_pos_.second);
        is_relative_ = false;
    }

    // calculate absolute position from message
    msg->pose.pose.position.x -= initial_pos_.first;
    msg->pose.pose.position.y -= initial_pos_.second;

    // Message: time; measured L/R; velocity lin/ang, position x/y
    PLOGD.printf("%.3f; %.3f; %.3f; %.3f; %.3f; %.3f",
                msg->twist.twist.linear.x, msg->twist.twist.angular.z,
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                odo_new.x, odo_new.y);

    pub_odom_->publish(*msg);
}

void
Motorcontroller::on_motor_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // subscriber to motor data
    /*
    Twist.msg
    -linear
    ---x  actual velocity motor left
    ---y  actual torque   motor left
    ---z  actual position motor left
    -angular
    ---x  actual velocity motor right
    ---y  actual torque   motor right
    ---z  actual position motor right
    */

    tmcm1638->parse_msg(msg);

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
    odom.twist.twist.linear.x = tmcm1638->get_linear_vel();
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.pose.pose.position.x = odo_new.x;
    odom.pose.pose.position.y = odo_new.y;
    odom.pose.pose.position.z = 0.142176;

    // Covariance -> 6x6 matrix
    for (uint8_t i = 0; i < 36; i += 7)
    {
        // set diagonal to variance
        odom.pose.covariance[i] = 0.3;
    }

    tf2::Quaternion q;
    // q.setRPY(0, 0, wh_fcruiser->get_orientation());
    q.setRPY(0, 0, 0);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Message: time; measured L/R; velocity lin/ang, position x/y
    PLOGD.printf("%.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.1f",
                msg->angular.x, msg->linear.x,
                tmcm1638->get_linear_vel(), tmcm1638->get_angular_vel(),
                odo_new.x, odo_new.y,
                tmcm1638->get_batt_voltage());

    pub_odom_->publish(odom);
}

void
Motorcontroller::publish_motor_cmd()
{
    std::string msg;

    geometry_msgs::msg::Quaternion m;
    if ((now().seconds() - cmd_vel_seconds_) > 1)
    {
        m = tmcm1638->cmd_vel(0.0, 0.0);
    }
    else
    {
        m = tmcm1638->cmd_vel(last_cmd_vel_.linear.x, last_cmd_vel_.angular.z);
    }
    
    PLOGD.printf("%.3f; %.3f; r: %.3f; l: %.3f", last_cmd_vel_.linear.x, last_cmd_vel_.angular.x, m.y, m.x);
    pub_motor_->publish(m);
}
}   // namespace sgd_hardware_drivers

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Motorcontroller>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
