#include "rclcpp/rclcpp.hpp"
#include "sgd_msgs/srv/get_global_plan.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_global_plan_client");
  rclcpp::Client<sgd_msgs::srv::GetGlobalPlan>::SharedPtr client =
    node->create_client<sgd_msgs::srv::GetGlobalPlan>("get_global_plan");

  auto request = std::make_shared<sgd_msgs::srv::GetGlobalPlan::Request>();
  //request->dest_id = "{100000200, 100004800, 100012300}";
  request->dest_id = "Berliner Tor 21";
  //request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Something was returned");
    
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_global_plan");
  }

  rclcpp::shutdown();
  return 0;
}