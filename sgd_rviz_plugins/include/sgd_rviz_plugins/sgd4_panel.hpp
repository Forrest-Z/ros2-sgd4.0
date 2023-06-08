#ifndef SGD_RVIZ_PLUGINS__SGD_PANEL_HPP_
#define SGD_RVIZ_PLUGINS__SGD_PANEL_HPP_

#include <QPushButton>
#include <QVBoxLayout>

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rviz_common/panel.hpp"

namespace sgd_rviz_plugins
{

class SGD4Panel : public rviz_common::Panel
{
    Q_OBJECT

private:
    /* data */
public:
    SGD4Panel(QWidget * parent = 0);
    ~SGD4Panel();

    // Override-able functions

    /**
     * Override to do initialization which depends on the DisplayContext being available.
     * The default implementation does nothing.
     */
    void onInitialize() override;

private Q_SLOTS:

    /**
     * @brief Click on button
     * 
     */
    void onStartClicked();


private:
    // The (non-spinning) client node used to invoke the action client
    rclcpp::Node::SharedPtr client_node_;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Point>> start_pub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>> sub_global_path_;

    void on_plan_received(nav_msgs::msg::Path::SharedPtr msg);


    // /// Return the name.
    // virtual QString getName() const;

    // /// Set the name.
    // virtual void setName(const QString & name);

    // /// Return a description of this Panel.
    // virtual QString getDescription() const;

    // /// Set a description of this Panel.
    // /**
    //  * Called by the factory which creates it.
    //  */
    // virtual void setDescription(const QString & description);

    // /// Return the class identifier which was used to create this instance.
    // /**
    //  * This version just returns whatever was set with setClassId().
    //  */
    // virtual QString getClassId() const;

    // /// Set the class identifier used to create this instance.
    // /**
    //  * Typically this will be set by the factory object which created it.
    //  */
    // virtual void setClassId(const QString & class_id);

    // /// Override to load configuration data.
    // /**
    //  * This version loads the name of the panel.
    //  */
    // virtual void load(const Config & config);

    // /// Override to save configuration data.
    // /**
    //  * This version saves the name and class ID of the panel.
    //  */
    // virtual void save(Config config) const;
};
    
} // namespace sgd_rviz_plugins

#endif