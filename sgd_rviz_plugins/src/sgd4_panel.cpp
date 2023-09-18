#include "sgd_rviz_plugins/sgd4_panel.hpp"

namespace sgd_rviz_plugins
{

SGD4Panel::SGD4Panel(QWidget * parent) 
        : Panel(parent)
{
    //setObjectName("Displays/SGD4Panel");

    // create start/stop button
    QPushButton * start_button = new QPushButton("Start");
    start_button->setObjectName("SgdPanel/StartButton");
    start_button->setShortcut(QKeySequence(QString("Ctrl+N")));
    start_button->setToolTip("Start / Stop the robot");
    
    QObject::connect(start_button, SIGNAL(clicked(bool)), this, SLOT(onStartClicked()));

    // add buttons to layout
    auto button_layout = new QVBoxLayout;
    button_layout->addWidget(start_button);

    // add layout to panel
    setLayout(button_layout);

    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args --remap __node:=navigation_dialog_action_client"});
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    // publisher for goal pose
    start_pub_ = client_node_->create_publisher<geometry_msgs::msg::Point>(
        "/goalpose", rclcpp::QoS(1).transient_local());

    // subscriber for computed path
    sub_global_path_ = client_node_->create_subscription<nav_msgs::msg::Path>(
        "/global_plan_sgd", rclcpp::QoS(1).transient_local(),
        std::bind(&SGD4Panel::on_plan_received, this, std::placeholders::_1));
}

SGD4Panel::~SGD4Panel() {}

void
SGD4Panel::onInitialize()
{
    // do something here
}

void
SGD4Panel::onStartClicked()
{
    if (global_plan_.poses.empty())
    {
        std::cout << "No global plan received";
    }
    geometry_msgs::msg::Point p;
    p.x = 10.234;
    p.y = 1.123;

    start_pub_->publish(p);
}

void
SGD4Panel::on_plan_received(nav_msgs::msg::Path::SharedPtr msg)
{
    // save received position
    std::cout << "Received position: " << msg->poses.back().pose.position.x << ", " << msg->poses.back().pose.position.y << "\n";
    global_plan_ = *msg;
}

} // namespace sgd_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(sgd_rviz_plugins::SGD4Panel, rviz_common::Panel)