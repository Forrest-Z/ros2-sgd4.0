
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

namespace sgd_lifecycle
{
    
class ExitPublisher : public rclcpp::Node
{
private:
    /* data */
public:
    ExitPublisher();
    ~ExitPublisher();
};



} // namespace sgd_lifecycle
