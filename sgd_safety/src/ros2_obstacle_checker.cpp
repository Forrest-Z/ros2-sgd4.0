#include "sgd_safety/ros2_obstacle_checker.hpp"

#define deg2rad(ang) (ang * M_PI / 180)
#define rad2deg(ang) (ang * 180 / M_PI)

namespace sgd_safety
{
    using std::placeholders::_1;

    Ros2_Obstacle_Checker::Ros2_Obstacle_Checker()
        : rclcpp_lifecycle::LifecycleNode("lidar_obstacle_checker")
    {
        RCLCPP_DEBUG(get_logger(), "Creating");

        // Add parameters
        declare_parameter("scan_topic", rclcpp::ParameterValue("scan"));
        declare_parameter("cmd_vel_contr_topic", rclcpp::ParameterValue("cmd_vel"));
        declare_parameter("sgd_move_topic", rclcpp::ParameterValue("cmd_vel_lidar"));

        declare_parameter("robot_width", rclcpp::ParameterValue(0.73));
        declare_parameter("distance_min", rclcpp::ParameterValue(0.5));
        declare_parameter("distance_max", rclcpp::ParameterValue(1.5));
    }

    Ros2_Obstacle_Checker::~Ros2_Obstacle_Checker()
    {
        // Destroy
    }

    CallbackReturn
    Ros2_Obstacle_Checker::on_configure(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Configure");

        // init parameters
        get_parameter("scan_topic", scan_topic_);
        get_parameter("cmd_vel_contr_topic", cmd_vel_contr_topic_);
        get_parameter("sgd_move_topic", cmd_vel_topic_);

        get_parameter("robot_width", robot_width_);
        get_parameter("distance_min", distance_min_);
        get_parameter("distance_max", distance_max_);

        init_pub_sub();

        try {
            oc.initialize(robot_width_, distance_min_, distance_max_);
        } catch (const std::invalid_argument & ia)
        {
            RCLCPP_ERROR(get_logger(), "%c", ia.what());
            return CallbackReturn::FAILURE;
        }
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_Obstacle_Checker::on_activate(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Activate");

        pub_cmd_vel->on_activate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_Obstacle_Checker::on_deactivate(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Deactivate");
        pub_cmd_vel->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_Obstacle_Checker::on_cleanup(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Cleanup");
        pub_cmd_vel.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_Obstacle_Checker::on_shutdown(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Shutdown");
        return CallbackReturn::SUCCESS;
    }

    void
    Ros2_Obstacle_Checker::init_pub_sub()
    {
        pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos);

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, default_qos, std::bind(&Ros2_Obstacle_Checker::on_scan_received, this, _1));
        sub_cmd_vel_contr_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_contr_topic_, default_qos, std::bind(&Ros2_Obstacle_Checker::on_cmd_vel_contr_received, this, _1));
    }

    void
    Ros2_Obstacle_Checker::on_cmd_vel_contr_received(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_ = *msg;
        cmd_vel_time = now().seconds();
    }

    void
    Ros2_Obstacle_Checker::on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // compute speed
        double speed = oc.compute_speed(msg->ranges, msg->angle_min, msg->angle_increment);

        /* After it has been verified that no obstacles interfere with the robot's trajectory, the calculated speed is set  */
        last_cmd_vel_.linear.x *= speed;
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel = last_cmd_vel_;
        if (speed == 0)
        {
            cmd_vel.angular.z = 0.0;
            cmd_vel.linear.x = 0.0;
            pub_cmd_vel->publish(cmd_vel);
        }
        else if ((now().seconds() - cmd_vel_time) < 1 && speed < 1.0)   // If the time elapsed is smaller than 1 second and the speed is smaller than 1 (no obstacles)
        {
            pub_cmd_vel->publish(cmd_vel);
        }
    }

} // namespace sgd_safety

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = std::make_shared<sgd_safety::Ros2_Obstacle_Checker>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar obstacle checker startup completed.");

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
