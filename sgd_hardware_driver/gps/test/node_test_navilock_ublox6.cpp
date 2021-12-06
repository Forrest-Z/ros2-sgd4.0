// Copyright 2021 HAW Hamburg
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

#include <chrono>
#include <future>
#include <cinttypes>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "gps/navilock_ublox6_gps.hpp"

using namespace std::chrono_literals;

template<typename DurationT>
void wait_for_future(
  rclcpp::Executor & executor,
  std::shared_future<void> & future,
  const DurationT & timeout)
{
  using rclcpp::FutureReturnCode;
  rclcpp::FutureReturnCode future_ret;
  auto start_time = std::chrono::steady_clock::now();
  future_ret = executor.spin_until_future_complete(future, timeout);
  auto elapsed_time = std::chrono::steady_clock::now() - start_time;
  EXPECT_EQ(FutureReturnCode::SUCCESS, future_ret) <<
    "future failed to be set after: " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
    " milliseconds\n";
}

TEST(NodeTestNavilockUBlox6, NodeStartup)
{
    std::string topic_pub = "dev_novalue";
    std::string topic_sub = "gps";
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    std::cout << "Run node test navilock\n";

    auto node = rclcpp::Node::make_shared("test_subscription");
    auto publisher = node->create_publisher<sgd_msgs::msg::Serial>(topic_pub, default_qos);

    std::cout << "Publisher node created\n";

    int counter = 0;
    std::promise<void> sub_called;
    std::shared_future<void> sub_called_future(sub_called.get_future());
    auto fail_after_timeout = 5s;
    auto callback =
        [&counter, &sub_called](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) -> void
        {
            ++counter;
            // check data
            sub_called.set_value();
        };

    sgd_msgs::msg::Serial msg;
    msg.msg = "$GPGGA,105353.000,5340.9773,N,00943.7816,E,1,08,0.9,15.2,M,45.1,M,,0000*64";
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::cout << "Start test cycle\n";
    {
        auto subscriber = node->create_subscription<sensor_msgs::msg::NavSatFix>(topic_sub, default_qos, callback);

        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using std::chrono::steady_clock;
        
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1);
        std::chrono::microseconds sleep_period = std::chrono::seconds(1);

        auto start = steady_clock::now();
        microseconds time_slept(0);
        
        // Wait for subscriber
        while (node->count_subscribers(topic_sub) <= 0 && time_slept < duration_cast<microseconds>(timeout))
        {
            rclcpp::Event::SharedPtr graph_event = node->get_graph_event();
            node->wait_for_graph_change(graph_event, sleep_period);
            time_slept = duration_cast<std::chrono::microseconds>(steady_clock::now() - start);
        }

        int64_t time_slept_count = std::chrono::duration_cast<std::chrono::microseconds>(time_slept).count();
        printf(
            "Waited %" PRId64 " microseconds for the subscriber to connect to topic '%s'\n",
            time_slept_count, topic_sub.c_str());

        msg.msg = "$GPGGA,105353.000,5340.9773,N,00943.7816,E,1,08,0.9,15.2,M,45.1,M,,0000*64";
        publisher->publish(msg);
        ASSERT_EQ(0, counter);

        // spin until the subscription is called or a timeout occurs
        printf("spin_until_future_complete(sub_called_future) - callback (1) expected\n");
        wait_for_future(executor, sub_called_future, fail_after_timeout);
        ASSERT_EQ(1, counter);

        // no additional calls to the subscription should be pending here
        printf("spin_once(nonblocking) - no callback expected\n");
        executor.spin_once(0s);
        ASSERT_EQ(1, counter);
        printf("spin_some() - no callback expected\n");
        executor.spin_some();
        ASSERT_EQ(1, counter);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<sgd_hardware::Navilock_UBlox6_GPS>();
    node->activate();
    //rclcpp::spin(node->get_node_base_interface());
    
    // Activate node
    std::cout << "Node is active\n";
    bool all_successful = RUN_ALL_TESTS();

    rclcpp::shutdown();

    return all_successful;
}