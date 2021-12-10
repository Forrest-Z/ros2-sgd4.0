#include "gtest/gtest.h"

#include "sgd_test/result_reader.hpp"
#include "sgd_safety/lidar_obstacle_checker.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace sgd_safety
{

using std::placeholders::_1;

class Test_Lidar_Obstacle_Checker_Integ : public rclcpp::Node
{
private:
    std::string result_filename;

    sgd_test::ResultReader<geometry_msgs::msg::Twist> result_reader;

    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
public:
    Test_Lidar_Obstacle_Checker_Integ();
    ~Test_Lidar_Obstacle_Checker_Integ();

    void on_msg_received(const geometry_msgs::msg::Twist::SharedPtr msg);
};

Test_Lidar_Obstacle_Checker_Integ::Test_Lidar_Obstacle_Checker_Integ()
    : rclcpp::Node("test_lidar_obstacle_checker_integ")
{
    result_filename = declare_parameter(result_filename, "laserscan.csv");
    result_reader.set_result_file(result_filename);

    auto obj = std::make_shared<sgd_safety::Lidar_Obstacle_Checker>();
    obj->activate();

    geometry_msgs::msg::Twist tw1;
    tw1.linear.x = 1.0;

    geometry_msgs::msg::Twist tw2;
    tw2.linear.x = 2.0;

    if (tw1 == tw2)
    {
        
    }

    // init subscriber
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>("topic_name", default_qos,
        std::bind(&on_msg_received, this, _1));
}

Test_Lidar_Obstacle_Checker_Integ::~Test_Lidar_Obstacle_Checker_Integ()
{
}

void
Test_Lidar_Obstacle_Checker_Integ::on_msg_received(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // get expected results from result reader
    auto res = result_reader.get_next();

    geometry_msgs::msg::Twist expect;
    expect.linear.x = std::stod(res[0]);
    expect.linear.y = std::stod(res[2]);
    expect.linear.z = std::stod(res[4]);
    expect.angular.x = std::stod(res[6]);
    expect.angular.y = std::stod(res[8]);
    expect.angular.z = std::stod(res[10]);

    EXPECT_EQ(msg->linear.x, expect.linear.x);
    // compute message from expected results
    // compare expected and received message
}

}

TEST(LidarObstacleTest, testNoStop)
{
    auto obj = std::make_shared<sgd_safety::Lidar_Obstacle_Checker>();
    obj->activate();
    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    rclcpp::init(argc, argv);

    bool all_successful = RUN_ALL_TESTS();

    //auto node = std::make_shared<sgd_safety::Test_Lidar_Obstacle_Checker_Integ>();
    //rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}