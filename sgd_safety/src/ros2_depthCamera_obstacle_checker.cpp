#include "sgd_safety/ros2_depthCamera_obstacle_checker.hpp"


namespace sgd_safety
{
    using std::placeholders::_1;

    Ros2_DepthCamera_Obstacle_Checker::Ros2_depthCamera_Obstacle_Checker()
        : rclcpp_lifecycle::LifecycleNode("depthCamera_obstacle_checker")
    {
        RCLCPP_DEBUG(get_logger(), "Creating");

        // Add parameters
        /* declare_parameter("scan_topic", rclcpp::ParameterValue("scan"));
        declare_parameter("cmd_vel_contr_topic", rclcpp::ParameterValue("cmd_vel"));
        declare_parameter("sgd_move_topic", rclcpp::ParameterValue("cmd_vel_lidar")); */

        declare_parameter("robot_width", rclcpp::ParameterValue(0.73));
        declare_parameter("distance_min", rclcpp::ParameterValue(0.5));
        declare_parameter("distance_max", rclcpp::ParameterValue(1.5));
    }

    Ros2_DepthCamera_Obstacle_Checker::~Ros2_DepthCamera_Obstacle_Checker()
    {
        // Destroy
    }

    CallbackReturn
    Ros2_DepthCamera_Obstacle_Checker::on_configure(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Configure");

        // init parameters
        /* get_parameter("scan_topic", scan_topic_);
        get_parameter("cmd_vel_contr_topic", cmd_vel_contr_topic_);
        get_parameter("sgd_move_topic", cmd_vel_topic_); */

        get_parameter("robot_width", robot_width_);
        get_parameter("distance_min", distance_min_);
        get_parameter("distance_max", distance_max_);

        init_pub_sub();

        /*
        try {
            dcam_oc.initialize(robot_width_, distance_min_, distance_max_);
        } catch (const std::invalid_argument & ia)
        {
            RCLCPP_ERROR(get_logger(), "%c", ia.what());
            return CallbackReturn::FAILURE;
        }
        */
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_DepthCamera_Obstacle_Checker::on_activate(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Activate");

        pub_cmd_vel->on_activate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_DepthCamera_Obstacle_Checker::on_deactivate(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Deactivate");
        pub_cmd_vel->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_DepthCamera_Obstacle_Checker::on_cleanup(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Cleanup");
        pub_cmd_vel.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn
    Ros2_DepthCamera_Obstacle_Checker::on_shutdown(const rclcpp_lifecycle::State &state __attribute__((unused)))
    {
        RCLCPP_DEBUG(get_logger(), "Shutdown");
        return CallbackReturn::SUCCESS;
    }

    void
    Ros2_DepthCamera_Obstacle_Checker::init_pub_sub()
    {
        pub_depthCamera_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_depthCamera_topic_, default_qos);
    }

    void
    Ros2_DepthCamera_Obstacle_Checker::Ros2_DepthCamera_Publish_Speed(void)
    {
        
        char error_code = 0;
        double speed_fromCam;
        
        
    
        std::tie(error_code, speed_fromCam) = dcam_oc.get_depthcam_speed();
        depthCamera_cmd_vel.linear.x = speed_fromCam;
        if(speed_fromCam == 0)
            depthCamera_cmd_vel.angular.z = 0.0;
        if(speed_fromCam < 1)
                pub_cmd_vel->publish(depthCamera_cmd_vel);
        if(error_code != 's')
            RCLCPP_INFO(this->get_logger(), "Camera Error.");
        
    }

} // namespace sgd_safety

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = std::make_shared<sgd_safety::Ros2_DepthCamera_Obstacle_Checker>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth Camera obstacle checker startup completed.");

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
