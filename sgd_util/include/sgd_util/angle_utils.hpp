#include <cmath>

namespace sgd_util
{

/**
 * @brief Ich stehe an Punkt p1(x1,y1) und schaue in Richtung p2(x2,y2). Die Funktion
 * gibt den entsprechenden Winkel zurück. Die x-Achse hat den Winkel 0.
 * 
 * @param x1 current x position
 * @param y1 current y position
 * @param x2 next x position
 * @param y2 next y position
 * @return double angle
 */
double angle(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1, x2-x1);
}

/**
 * @brief The function returns the interior angle between the segments S1(x1,y1 -> x2,y2) and S2(x2,y2 -> x3,y3).
 * 
 * @param x1 
 * @param y1 
 * @param x2 
 * @param y2 
 * @param x3 
 * @param y3 
 * @return double 
 */
double delta_angle(double x1, double y1, double x2, double y2, double x3, double y3)
{
    auto ang1 = angle(x2,y2,x1,y1);
    auto ang2 = angle(x2,y2,x3,y3);
    
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

} // namespace sgd_util

