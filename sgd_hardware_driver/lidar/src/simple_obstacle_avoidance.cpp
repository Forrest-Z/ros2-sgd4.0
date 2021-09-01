#include "lidar/simple_obstacle_avoidance.hpp"

namespace sgd_sensors
{

using namespace std::chrono_literals;   // if a timer is used

Simple_Obstacle_Avoidance::Simple_Obstacle_Avoidance():
    nav2_util::LifecycleNode("example_node", "", true)
{
    RCLCPP_INFO(get_logger(), "Creating");

    add_parameter("example_param", rclcpp::ParameterValue("example"));
}

Simple_Obstacle_Avoidance::~Simple_Obstacle_Avoidance()
{
    // Destroy
}

nav2_util::CallbackReturn
Simple_Obstacle_Avoidance::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Obstacle_Avoidance::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Activating");

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Obstacle_Avoidance::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Obstacle_Avoidance::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Cleanup");
    sub_lidar_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Simple_Obstacle_Avoidance::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Simple_Obstacle_Avoidance::init_parameters()
{
    //get_parameter("example_param", example_param_);
}

void
Simple_Obstacle_Avoidance::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init publisher and subscriber");
    sub_lidar_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", default_qos, std::bind(&Simple_Obstacle_Avoidance::on_lidar_received, this, std::placeholders::_1));
}

void
Simple_Obstacle_Avoidance::on_lidar_received(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_sensors::Simple_Obstacle_Avoidance>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
