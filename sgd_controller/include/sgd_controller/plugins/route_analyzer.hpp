#include <string>
#include <list>
#include <vector>
#include <cmath>
#include <map>
#include <iostream>

namespace sgd_ctrl
{

class RouteAnalyzer
{

struct RouteInfo
{
    double angle;
    double distance;
};

struct Pos2D
{
    double x, y;
    double angle;
};

private:
    std::list<Pos2D> global_plan_;
    double last_dist = 10.0;    // last distance to next waypoint

    double delta_angle(double x1, double y1, double x2, double y2, double x3, double y3);
public:
    RouteAnalyzer(/* args */);
    ~RouteAnalyzer();

    /**
     * @brief 
     * 
     * @param waypoints 
     */
    void add_waypoints(std::vector<std::pair<double, double>> waypoints);

    /**
     * @brief 
     * 
     * @param pos_x 
     * @param pos_y 
     * @return RouteInfo 
     */
    RouteInfo info(double pos_x, double pos_y);

    /**
     * @brief Set next waypoint
     */
    void next_wp();

};

RouteAnalyzer::RouteAnalyzer() {}

RouteAnalyzer::~RouteAnalyzer() {}

void
RouteAnalyzer::add_waypoints(std::vector<std::pair<double, double>> waypoints)
{
    if (waypoints.size() < 2)   // minimum size is 2
    {
        return;
    }

    // Angle for first waypoint is 0
    Pos2D pos;
    pos.x = waypoints.front().first;
    pos.y = waypoints.front().second;
    pos.angle = 0.0;
    global_plan_.push_back(pos);

    for (std::size_t i = 1; i < (waypoints.size()-1); i++)
    {
        pos.x = waypoints.at(i).first;
        pos.y = waypoints.at(i).second;
        
        // calculate angle
        pos.angle = delta_angle(waypoints.at(i-1).first, waypoints.at(i-1).second,
                                          waypoints.at(i).first  , waypoints.at(i).second,
                                          waypoints.at(i+1).first, waypoints.at(i+1).second) - M_PI;
        global_plan_.push_back(pos);
    }

    Pos2D pos_end;
    pos_end.x = waypoints.back().first;
    pos_end.y = waypoints.back().second;
    pos_end.angle = 3.5;
    global_plan_.push_back(pos_end);
}

RouteAnalyzer::RouteInfo
RouteAnalyzer::info(double pos_x, double pos_y)
{
    // if distance < 1.0m and distance > last_dist -> remove current waypoint
    auto wp = global_plan_.front();
    RouteInfo info;
    info.angle = wp.angle;
    info.distance = std::hypot(wp.x - pos_x, wp.y - pos_y);
    return info;
}

void
RouteAnalyzer::next_wp()
{
    // remove first waypoint
    global_plan_.pop_front();
}

double
RouteAnalyzer::delta_angle(double x1, double y1, double x2, double y2, double x3, double y3)
{
    auto ang1 = atan2(y1-y2, x1-x2);
    auto ang2 = atan2(y3-y2, x3-x2);
    
    if (ang1 < 0)   ang1 = ang1+2*M_PI;
    if (ang2 < 0)   ang2 = ang2+2*M_PI;    

    double diff_ang = abs(ang2 - ang1);

    std::cout << "Winkel 1: " << ang1 << ", Winkel 2: " << ang2 << std::endl;

    if (diff_ang > M_PI)
    {
        return abs(ang2 - ang1 - 2*M_PI);
    } else
    {
        return diff_ang;
    }
}

} // namespace sgd_ctrl
