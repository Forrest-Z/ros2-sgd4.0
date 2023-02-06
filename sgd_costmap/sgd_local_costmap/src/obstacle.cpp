#include "sgd_local_costmap/obstacle.hpp"

namespace sgd_local_costmap
{
    
Obstacle::Obstacle(/* args */)
{
    id_ = 123;
}

Obstacle::~Obstacle()
{
}

void
Obstacle::add_vertex(float x, float y)
{
    vertices_.push_back(Point(x,y));
}


std::vector<Point>
Obstacle::expected_scan(float robo_x, float robo_y, float robo_yaw)
{
    std::vector<Point> exp_scan;
    // get Point with min phi and max phi
    int min_phi_pnt, max_phi_pnt;
    float min_phi, max_phi;
    for (int i = 0; i < vertices_.size(); i++)
    {
        float phi = vertices_[i].get_polar_phi(robo_x, robo_y, robo_yaw);
        if (phi < min_phi)
        {
            min_phi = phi;
            min_phi_pnt = i;
        }
        if (phi > max_phi)
        {
            max_phi = phi;
            max_phi_pnt = i;
        }
    }

    exp_scan.push_back(vertices_[min_phi_pnt]);
    // return if only one point is visible
    if (min_phi_pnt == max_phi_pnt)     return exp_scan;

    // get points in between
    auto pnt1 = get_vertex(min_phi_pnt+1);
    float phi1 = pnt1.get_polar_phi(exp_scan[0].x, exp_scan[0].y, min_phi);

    auto pnt2 = get_vertex(min_phi_pnt+1);
    float phi2 = pnt2.get_polar_phi(exp_scan[0].x, exp_scan[0].y, min_phi);

    int direction = phi1 > phi2 ? 1 : -1;
    do
    {
        exp_scan.push_back(get_vertex(min_phi_pnt + direction));
        direction += direction;
    } while (!get_vertex(max_phi_pnt).equals(exp_scan.back()));
    
    return exp_scan;
}

sgd_msgs::msg::VecObstacle
Obstacle::to_msg()
{
    // return obstacle converted to sgd message
    sgd_msgs::msg::VecObstacle msg;
    msg.id = id_;
    for (auto p : vertices_)
    {
        geometry_msgs::msg::Point32 pnt;
        pnt.x = p.x;
        pnt.y = p.y;
        pnt.z = 0.0;
        msg.vertices.push_back(pnt);
    }
    return msg;
}

} // namespace sgd_local_costmap