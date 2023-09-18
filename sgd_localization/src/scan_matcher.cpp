// Copyright 2022 HAW Hamburg
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

#include "sgd_localization/scan_matcher.hpp"

namespace sgd_localization
{

ScanMatcher::ScanMatcher() :
    rclcpp_lifecycle::LifecycleNode("scan_matcher")
{
    PLOGD << "Creating...";

    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    declare_parameter("scan_topic", rclcpp::ParameterValue("/scan"));

    // initialize logging
    std::string log_dir_, log_sev_;
    get_parameter("log_dir", log_dir_);
    get_parameter("log_severity", log_sev_);
    std::string log_file(log_dir_ + "/scan_matcher.txt");
    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created scan_matcher node. PLOG logging severity is %s", log_sev_.c_str());
    RCLCPP_INFO(get_logger(), "Created scan_matcher node. Save log file to %s", log_file.c_str());
}

ScanMatcher::~ScanMatcher()
{
    // Destroy
}

CallbackReturn
ScanMatcher::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Configuring...";
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize parameters, pub/sub, services, etc.
    //init_parameters();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ScanMatcher::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Activating...";

    // Subscriber
    auto scan_topic = get_parameter("scan_topic").as_string();
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, default_qos,
            std::bind(&ScanMatcher::on_scan_received, this, std::placeholders::_1));
    PLOGI.printf("Subscribe to '%s'", scan_topic.c_str());

    std::string local_map_topic = "local_map";
    sub_local_map_ = this->create_subscription<sgd_msgs::msg::VecObstacleArray>(local_map_topic, default_qos,
            std::bind(&ScanMatcher::on_local_map_received, this, std::placeholders::_1));

    // Publisher
    std::string visual_topic = "/obstacles/vec";
    pub_obstacles = this->create_publisher<sgd_msgs::msg::VecObstacleArray>(visual_topic, default_qos);
    pub_obstacles->on_activate();
    PLOGI.printf("Create publisher on topic '%s'", visual_topic.c_str());

    // Publisher
    std::string orientation_topic = "scan_orientation";
    pub_orientation = this->create_publisher<geometry_msgs::msg::Quaternion>(orientation_topic, default_qos);
    pub_orientation->on_activate();
    PLOGI.printf("Create publisher on topic '%s'", orientation_topic.c_str());

    obstacle_id = 0;
    last_msg = now();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ScanMatcher::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Deactivating...";
    // pub_obstacles->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ScanMatcher::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Cleaning up...";
    return CallbackReturn::SUCCESS;
}

CallbackReturn
ScanMatcher::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Shutting down...";
    return CallbackReturn::SUCCESS;
}

// void
// ScanMatcher::init_parameters()
// {
//     get_parameter("scan_topic", scan_topic);
// }

void
ScanMatcher::on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // process message every 5 seconds
    if ((now()-last_msg).nanoseconds() < 5E9)  return;
    last_msg = now();

    // get current position of robo
    try
    {
        tf_map_scan = tf_buffer_->lookupTransform("map", "base_scan", rclcpp::Time(0));
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Could not get transform from map to base_scan");
    }

    RCLCPP_INFO(get_logger(), "New scan: -------------");
    PLOGI << "New scan: --------------";
    print_debug_ = true;

    sgd_msgs::msg::VecObstacleArray obstacle_message;
    obstacle_message.header.stamp = now();
    obstacle_message.header.frame_id = "map";
    obstacle_message.pose.position.x = tf_map_scan.transform.translation.x;
    obstacle_message.pose.position.y = tf_map_scan.transform.translation.y;
    obstacle_message.pose.orientation = tf_map_scan.transform.rotation;
    obstacle_message.obstacles.clear();

    float yaw = tf2::getYaw(tf_map_scan.transform.rotation);

    // auto compare = [](Line a, Line b) { return a.length() < b.length(); };
    // std::priority_queue<Line, std::vector<Line>, decltype(compare)> lines(compare);
    std::vector<Line> lines_visual;

    float factor = 1.0;
    float angle_diff = 0.0;
    int subset_start = 0;  // set possible start point for subset
    int num_inv_pnts = 0;   // number of consecutive invalid scan ranges
    for (int i = 1; i < msg->ranges.size(); i++)
    {
        // check if point is in the valid range
        if (msg->ranges[i-1] > msg->range_max || msg->ranges[i-1] < msg->range_min)
        {
            num_inv_pnts += 1;
            if (num_inv_pnts > 3)
            {
                // start new subset
                subset_start = i;
            }
        }
        else if (num_inv_pnts > 0)
        {
            num_inv_pnts -= 1;
        }

        // jump in data points detected
        if (abs(msg->ranges[i] - msg->ranges[i-1]) > 1.0 || i+1 >= msg->ranges.size())
        {
            if (i - subset_start < 5)   // subset should have at least 5 data points
            {
                // forget this subset
                subset_start = i;
                num_inv_pnts = 0;
                continue;
            }

            // create new vector with subset in x-y-coordinates
            PLOGI << "New subset detected (size=" << i-subset_start << "):";
            std::vector<Point> subset;
            for (int k = subset_start; k < i; k++)
            {
                if (is_valid(msg->ranges[k]))
                {
                    // Achtung: Lidar-Sensor ist kopfüber angebracht
                    float ang = -k*msg->angle_increment - msg->angle_min;
                    Point p(cos(ang)*msg->ranges[k], sin(ang)*msg->ranges[k]);
                    subset.push_back(p);
                }
            }
            // compute ausgleichsgerade
            if (subset.size() < 1)  continue;
            auto points = split_and_merge(subset, 0, subset.size(), 0);

            // Obstacle obst;
            // add points to obstacle
            PLOGI << "Computed lines:";
            for (int i = 1; i < points.size(); i++)
            {
                auto scan_pnts = points[i].second - points[i-1].second;
                if (scan_pnts < 10)  continue;
                
                Line ln(
                    points[i-1].first.to_frame(tf_map_scan.transform.translation.x, tf_map_scan.transform.translation.y, yaw),
                    points[i].first.to_frame(tf_map_scan.transform.translation.x, tf_map_scan.transform.translation.y, yaw)
                );

                PLOGI.printf("Add line: (%.2f, %.2f) - (%.2f, %.2f) length: %.3fm",
                        ln.p1_.x, ln.p1_.y, ln.p2_.x, ln.p2_.y, ln.length());

                auto nrst = get_nearest_from_local(ln);
                if (nrst.p1_.x == 0.0 && nrst.p1_.y == 0.0 && nrst.p2_.x == 0.0 && nrst.p2_.y == 0.0)   continue;

            // 3. compute Winkelversatz
                auto ang = ln.angle(nrst);
            // 4. aktualisiere Winkelversatz mit Wahrscheinlichkeit berechnet aus:
                // 4.1 Abstand zum Fahrzeug - TODO
                // 4.2 Abstand der beiden Linien zueinander - TODO
                // 4.3 Verwendete Anzahl an Scan-Punkten - TODO
                
                angle_diff += ang * scan_pnts/100.0F;
                factor *= scan_pnts/100.0F;

                lines_visual.push_back(ln);
                lines_visual.push_back(nrst);
            }
            subset_start = i;
        }
    }
    if (lines_visual.empty())   return;
    // Publish orientation
    angle_diff = angle_diff / factor;
    angle_diff = std::max(-max_angle_diff, std::min(angle_diff, max_angle_diff/2));
    angle_diff = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle_diff);
    RCLCPP_INFO(get_logger(), "Orientation difference: %.2f", angle_diff);
    geometry_msgs::msg::Quaternion orientation_;
    orientation_ = tf2::toMsg(q);
    pub_orientation->publish(orientation_);

    // generate message to display lines
    for (auto ln : lines_visual)
    {
        sgd_msgs::msg::VecObstacle obst;
        obst.id = 123;
        geometry_msgs::msg::Point32 pnt;
        pnt.x = ln.p1_.x;
        pnt.y = ln.p1_.y;
        pnt.z = 0.0;
        obst.vertices.push_back(pnt);
        pnt.x = ln.p2_.x;
        pnt.y = ln.p2_.y;
        pnt.z = 0.0;
        obst.vertices.push_back(pnt);

        obstacle_message.obstacles.push_back(obst);
    }
    pub_obstacles->publish(obstacle_message);
}

void
ScanMatcher::on_local_map_received(const sgd_msgs::msg::VecObstacleArray::SharedPtr msg)
{
    // save obstacles from local map
    local_obstacles_ = *msg;
}

std::vector<std::pair<Point, int>>
ScanMatcher::split_and_merge(std::vector<Point> subset, int start, int end, int depth)
{
    // implementation of a split-and-merge algorithm by Douglas and Peucker

    // if end-start < 2 -> two or less points in subset
    PLOGD.printf("Split and merge(start=%d, end=%d, depth=%d)", start, end, depth);
    // compute average x and y coordinate
    float x_a = 0.0, y_a = 0.0;
    for (int m = start; m < end; m++)
    {
        x_a += subset.at(m).x;
        y_a += subset.at(m).y;
    }
    x_a /= (end-start);
    y_a /= (end-start);

    float sum_xiyi = 0.0, sum_xi2 = 0.0;
    float sroot = sqrt(pow(subset[end-1].x-subset[start].x,2) + pow(subset[end-1].y-subset[start].y,2));
    int max_dist_index = 0; // index of point with maximum distance to line
    float max_dist = 0.0;   // maximum distance to line
    for (int n = start; n < end; n++)
    {
        sum_xiyi += (subset[n].x-x_a)*(subset[n].y-y_a);
        sum_xi2 += std::pow(subset[n].x-x_a, 2);

        // get data point with maximum distance to line connecting first and last data point
        float dist = abs((subset[end-1].x-subset[start].x)*(subset[start].y-subset[n].y) 
                        -(subset[start].x-subset[n].x)*(subset[end-1].y-subset[start].y)) / sroot;
        if (dist > max_dist)
        {
            max_dist = dist;
            max_dist_index = n;
        }
    }

    if (max_dist > 0.5 && depth < 4)
    {
        // split line and do it again
        depth++;
        auto line1 = split_and_merge(subset, start, max_dist_index, depth); // first line
        auto line2 = split_and_merge(subset, max_dist_index, end, depth);   // second line

        // calculate intersection
        // replace last point of line 1 with intersection point
        auto p_intersect = calc_intersection(line1[line1.size() - 2].first, line1[line1.size() - 1].first,
                                             line2[0].first, line2[1].first);
        line1.pop_back();
        line1.push_back({p_intersect, max_dist_index});
        // add line2 to line1
        for (int l = 1; l<line2.size(); l++)
        {
            line1.push_back(line2[l]);
        }

        return line1;
    }

    std::vector<std::pair<Point, int>> regression;
    float m = sum_xiyi / sum_xi2;
    float b = y_a - m*x_a;

    Point p1(subset.at(start).x, m*subset.at(start).x+b);
    Point p2(subset.at(end-1).x, m*subset.at(end-1).x+b);

    regression.push_back({p1, start});
    regression.push_back({p2, end-1});

    // print points and regression
    PLOGI << "Return Regression: " << p1.x << ", " << p1.y << "; " << p2.x << ", " << p2.y;
    print_debug_ = false;

    return regression;
}

Point
ScanMatcher::calc_intersection(Point p1, Point p2, Point p3, Point p4)
{
    // TODO check if steigung is identical -> kann logisch eigentlich nicht passieren
    float u = (p1.x*(p3.y-p2.y) + p2.x*(p1.y-p3.y) + p3.x*(p2.y-p1.y))
            / (p1.x*(p3.y-p4.y) + p2.x*(p4.y-p3.y) + p3.x*(p2.y-p1.y) + p4.x*(p1.y-p2.y));
    Point ip(p3.x+u*(p4.x-p3.x), p3.y+u*(p4.y-p3.y));   // intersection point

    return ip;
}

float
ScanMatcher::calc_distance_to_line(Point pl1, Point pl2, Point p)
{
    // TODO: betrachte Distanz, wenn Punkt außerhalb der durch die beiden Punkte gebildeten Geraden liegt
    // Wenn Abstand pl1->p und Abstand pl2->p größer als Abstand zu Linie, dann return kleinsten Abstand zu pl1 oder pl2
    float dist_line = abs((pl2.x-pl1.x)*(pl1.y-p.y)-(pl1.x-p.x)*(pl2.y-pl1.y)) / sqrt(pow(pl2.x-pl1.x,2) + pow(pl2.y-pl1.y,2));
    float dist_pl1 = std::hypotf(pl1.x-p.x, pl1.y-p.y);
    float dist_pl2 = std::hypotf(pl2.x-p.x, pl2.y-p.y);

    return (dist_pl1 > dist_line && dist_pl2 > dist_line) ? std::min(dist_pl1, dist_pl2) : dist_line;
}

Line
ScanMatcher::get_nearest_from_local(Line l)
{
    float nearest = 10.0;
    float pnts[] = {0.0, 0.0, 0.0, 0.0};
    
    for (auto o : local_obstacles_.obstacles)
    {
        for (int i = 0; i < o.vertices.size(); i++)
        {
            // check distance for obstacles
            Point p0(o.vertices[i<1 ? o.vertices.size()-1 : i-1]);
            Point p00(o.vertices[i]);

            float dist1 = calc_distance_to_line(p0, p00, l.p1_);
            float dist2 = calc_distance_to_line(p0, p00, l.p2_);
            PLOGD.printf("Calculated distances: %.3f & %.3f", dist1, dist2);
            // entscheide, ob Linie weiter betrachtet wird
            if (dist1 + dist2 < nearest)
            {
                PLOGD << "Set return line";
                nearest = dist1 + dist2;
                pnts[0] = p0.x;
                pnts[1] = p0.y;
                pnts[2] = p00.x;
                pnts[3] = p00.y;
            }
        }
    }
    return Line(Point(pnts[0], pnts[1]), Point(pnts[2], pnts[3]));
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_localization::ScanMatcher>(); 
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}